/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <linux/videodev2.h>
#include <vector>
#include <cmath>
#include <malloc.h>
#include <thread> 
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "umap_util.hpp"
#include "wp_planning.hpp"

#include "get_image.hpp"
extern "C" {
#include "camera.h"
}
using namespace std;
using namespace umap_utility;
using namespace wp;
using namespace cv;

typedef mavros_msgs::GlobalPositionTarget gbpos;
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

sensor_msgs::NavSatFix current_pose;
void pose_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    current_pose = *msg;
}

bool checkEqualPose(const mavros_msgs::GlobalPositionTarget expectedPosition)
{
    double r = 0.00005;
    // return (current_pose.latitude - expectedPosition.latitude)*(current_pose.latitude - expectedPosition.latitude) + (current_pose.longitude - expectedPosition.longitude)*(current_pose.longitude - expectedPosition.longitude) < r*r;
    return abs(current_pose.latitude - expectedPosition.latitude) < r && abs(current_pose.longitude - expectedPosition.longitude) < r;
}

bool atZeroLatLong()
{
    // check if it is around 0 0 point
    return abs(current_pose.latitude - 0.0) < std::numeric_limits<double>::epsilon() && abs(current_pose.longitude - 0.0) < std::numeric_limits<double>::epsilon();
}

void changeGlobalPoseWithRef(mavros_msgs::GlobalPositionTarget &global_pose, mavros_msgs::GlobalPositionTarget &global_home_pose, geometry_msgs::PoseStamped local_pose)
{
    // flat world conversion with 111,111 meter per degree
    global_pose.latitude = global_home_pose.latitude - local_pose.pose.position.y/111111.0; 
    global_pose.longitude = global_home_pose.longitude - local_pose.pose.position.x/111111.0;
    global_pose.altitude = global_home_pose.altitude;
}

int main(int argc, char **argv)
{
    enum
    {
        STEREO_BM = 0,
        STEREO_SGBM = 1,
        STEREO_HH = 2,
        STEREO_VAR = 3,
        STEREO_3WAY = 4
    };

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    std::vector<geometry_msgs::PoseStamped> local_poses(20);
    mavros_msgs::GlobalPositionTarget global_pose;
    mavros_msgs::GlobalPositionTarget global_home_pose;

    int64 t;
    char filename[50];

    double min, max;
    int i,j,k,current_x, current_y;
    int loop_count;
    int SADWindowSize, numberOfDisparities, sgbmWinSize, cn;
    Size image_size(2432, 1842);

    double f,b;

    int current_waypoint_index = 0;
    int num_wp = 0;
    
    bool armed = false;
    bool enabled = false;
    bool ready = false;

    int obj_count;
    int wp_gen = 10;
    vector<ellipse_desc> ellipse_list(50);
    vector< pair<double, double> > waypoints(200);
    vector< pair<double, double> > waypoints_pub(200);
    
    int framesize = image_size.width*image_size.height;

    Mat left_image(1842, 2432, CV_16UC1);
    Mat right_image(1842, 2432, CV_16UC1);
    Mat left_image_8uc1(1842, 2432, CV_8UC1);
    Mat right_image_8uc1(1842, 2432, CV_8UC1);
    Mat left_image_debayer(921, 1216, CV_8UC1);
    Mat right_image_debayer(921, 1216, CV_8UC1);
    Mat disp(600,800, CV_8UC3);
    Mat disp8(600,800, CV_8UC3);
    Mat obstacle_map(2000,6000, CV_8UC3);

    Mat R, T, R1, R2, P1, P2, Q;
    Mat cropped_left, cropped_right;
    Mat camera_matrix[2], dist_coeffs[2];
    Mat rimg0, rimg1;
    Mat rimg[2], cimg;
    Mat rmap[2][2];
    Rect validRoi[2];
    int VROIX, VROIY, VROIW, VROIH;
    int w, h;
    double sf;

    // Copy the data into an OpenCV Mat structure
    uint8_t *ptr_i, *ptr_i1, *dbptr;
    uint8_t *rptr_i, *rptr_i1, *rdbptr;
    int red,green,blue;
    int ib, jb;

    FileStorage fs("/home/ubuntu/.ros/intrinsics.yml", CV_STORAGE_READ);
    

    double GYb;
    double *pe1, *pe2, *se1, *se2;

    int alg = STEREO_SGBM;
    StereoSGBM sgbm;
    // Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

    char *dev_name = "/dev/video0";
	char *dev_name2 = "/dev/video1";
	char left_name[50];
	char right_name[50];

    // Setup Intrinsics Camera Matrix
    if (fs.isOpened())
    {
        fs["M1"] >> camera_matrix[0];
        fs["M2"] >> camera_matrix[1];
        fs["D1"] >> dist_coeffs[0];
        fs["D2"] >> dist_coeffs[1];
        fs.release();
    }
    else
        ROS_INFO("Error: can not read the intrinsic parameters\n");

    fs.open("/home/ubuntu/.ros/extrinsics.yml", CV_STORAGE_READ);
    if (fs.isOpened())
    {
        fs["R"] >> R;
        fs["T"] >> T;
        fs["R1"] >> R1;
        fs["R2"] >> R2;
        fs["P1"] >> P1;
        fs["P2"] >> P2;
        fs["Q"] >> Q;
        fs["VROIX"] >> VROIX;
        fs["VROIY"] >> VROIY;
        fs["VROIW"] >> VROIW;
        fs["VROIH"] >> VROIH;

        // cout << R << T << R1 << R2 << P1 << P2 << Q << endl;
        fs.release();
    }
    else
        ROS_INFO("Error: can not read the intrinsic parameters\n");

	
    std::thread spi_thread(getimage::flushBuffer); // signal every 250 ms
	cameraState *c1 = init_camera(dev_name, 2432, 1842, 1, 3, 2);
	cameraState *c2 = init_camera(dev_name2, 2432, 1842, 1, 3, 2);

	int control_val, control_val2;
	v4l2SetControl(c1, V4L2_CID_EXPOSURE, 1);
	control_val = v4l2GetControl(c1, V4L2_CID_EXPOSURE);
	fprintf(stderr, "set value:%d\n", control_val);

	v4l2SetControl(c1, V4L2_CID_GAIN, 1);
	control_val = v4l2GetControl(c1, V4L2_CID_GAIN);
	fprintf(stderr, "set value:%d\n", control_val);

	v4l2SetControl(c2, V4L2_CID_EXPOSURE, 1);
	control_val2 = v4l2GetControl(c2, V4L2_CID_EXPOSURE);
	fprintf(stderr, "set value:%d\n", control_val2);

	v4l2SetControl(c2, V4L2_CID_GAIN, 1);
	control_val2 = v4l2GetControl(c1, V4L2_CID_GAIN);
	fprintf(stderr, "set value:%d\n", control_val2);

	// Set Contrast to 1
	v4l2SetControl(c1, V4L2_CID_CONTRAST, 1);
	control_val = v4l2GetControl(c1, V4L2_CID_CONTRAST);
	fprintf(stderr, "set value:%d\n", control_val);

	v4l2SetControl(c2, V4L2_CID_CONTRAST, 1);
	control_val2 = v4l2GetControl(c2, V4L2_CID_CONTRAST);
	fprintf(stderr, "set value:%d\n", control_val2);

	void *hostBuffer = malloc(c1->width * c1->height * c1->bytePerPixel * 5);
	void *hostBuffer2 = malloc(c2->width * c2->height * c2->bytePerPixel * 5);

	struct v4l2_buffer buff1;
	struct v4l2_buffer buff2;

	for (i = 0; i < 3; i++)
	{
		// remove image from buff first
		fprintf(stdout, "Flushing camera buffer\n");
		if(getBufferTimeOut(c1, &buff1, 1) == 1)
		{
			pushBuffer(c1, &buff1);
		}
		if(getBufferTimeOut(c2, &buff2, 1) == 1)
		{
			pushBuffer(c2, &buff2);
		}
	}

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, pose_cb);
    
    ros::Publisher global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");



    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ROS_INFO("current state connected %d", current_state.connected);
        ros::spinOnce();
        ROS_INFO("SPIN!");
        rate.sleep();
    }

    bool trigger_new_guided = false;
    // while (current_state.mode != "GUIDED")
    // {
    //     ROS_INFO("Not in GUIDED MODE....Wait for position set up");
    //     ros::spinOnce();
    //     rate.sleep();
    //     trigger_new_guided = true;
    // }
    
    ROS_INFO("set pose array");

    while(atZeroLatLong())
    {
        // std::cout << current_pose.latitude << " " << current_pose.longitude << std::endl;
        ROS_INFO("check at zero point %.6f %.6f %.6f", current_pose.latitude, current_pose.longitude, current_pose.altitude);
        ros::spinOnce();
        rate.sleep();
    }

    global_home_pose.latitude = current_pose.latitude;
    global_home_pose.longitude = current_pose.longitude;
    global_home_pose.altitude = 4;
    global_home_pose.velocity.x = 2.0;
    global_home_pose.velocity.y = 2.0;
    global_home_pose.velocity.z = 2.0;
    global_home_pose.acceleration_or_force.x = 2;
    global_home_pose.acceleration_or_force.y = 2;
    global_home_pose.acceleration_or_force.z = 2;
    global_home_pose.coordinate_frame = gbpos::FRAME_GLOBAL_REL_ALT;
    global_home_pose.type_mask = 4088;

    global_pose.latitude = global_home_pose.latitude;
    global_pose.longitude = global_home_pose.longitude;
    global_pose.altitude = global_home_pose.altitude;
    global_pose.velocity.x = global_home_pose.velocity.x;
    global_pose.velocity.y = global_home_pose.velocity.y;
    global_pose.velocity.z = global_home_pose.velocity.z;
    global_pose.acceleration_or_force.x = global_home_pose.acceleration_or_force.x;
    global_pose.acceleration_or_force.y = global_home_pose.acceleration_or_force.y;
    global_pose.acceleration_or_force.z = global_home_pose.acceleration_or_force.z;
    global_pose.coordinate_frame = global_home_pose.coordinate_frame;
    global_pose.type_mask = global_home_pose.type_mask;

    // mock up waypoint
    for (i = 0; i < wp_gen ; i++)
    {
        local_poses[i].pose.position.x = 10.0*i; // in meter
        local_poses[i].pose.position.y = 0.0;
        local_poses[i].pose.position.z = 4.0;
    }

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        global_pos_pub.publish(global_home_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_calculation;
    
    f = 1065; // 
    b = 15;   // in centimeter

    loop_count = 0;
    last_calculation = ros::Time::now();
    // if(current_state.mode == "OFFBOARD") enabled = true;
    // if(current_state.armed) armed = true;
    while (ros::ok())
    {
        if (current_state.mode != "GUIDED")
        {   
            ROS_INFO("Not in GUIDED MODE");
            trigger_new_guided = false;
            enabled = false;
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        else
        {
            ROS_INFO("In GUIDED MODE");
            if (!trigger_new_guided)
            {
                // if re-trigger guided mode, set new global zero position
                global_home_pose.latitude = current_pose.latitude;
                global_home_pose.longitude = current_pose.longitude;
                global_home_pose.altitude = current_pose.altitude;

                global_pose.latitude = global_home_pose.latitude;
                global_pose.longitude = global_home_pose.longitude;
                global_pose.altitude = global_home_pose.altitude;

                current_waypoint_index = 0;
                trigger_new_guided = true;

            }
            enabled = true;
        }

        if (!enabled) {
            global_pos_pub.publish(global_home_pose);
            ros::spinOnce();
        }
        ready = true; // TODO REMOVE THIS!
        if (!ready && checkEqualPose(global_home_pose))
        {
            ready = true;
        }

        ROS_INFO("Current waypoint index %d", current_waypoint_index);
        ROS_INFO("Global lat %.6f %.6f", global_pose.latitude, global_pose.longitude);
        if(checkEqualPose(global_pose) && current_waypoint_index != 200)
        {
            current_waypoint_index++;
        }
        
        if(ready && ros::Time::now() - last_calculation > ros::Duration(0.5))
        {
            if(!enabled) continue;

            // get image and debayer
            getBuffer(c1, &buff1);
            getBuffer(c2, &buff2);

            memcpy(left_image.data, buff1.m.userptr, framesize*2);
            memcpy(right_image.data, buff2.m.userptr, framesize*2);
            
            pushBuffer(c1, &buff1);
            pushBuffer(c2, &buff2);
            
            // getting image
            // imwrite("/home/ubuntu/img_log/left_image_16C.jpg", left_image);
            t = getTickCount();
            left_image.convertTo(left_image_8uc1, CV_8UC1, 1);
            right_image.convertTo(right_image_8uc1, CV_8UC1, 1);

            sprintf(filename, "/home/ubuntu/img_log/left%d.jpg", loop_count);
            imwrite(filename, left_image_8uc1);

            sprintf( filename, "/home/ubuntu/img_log/right%d.jpg", loop_count);
            imwrite(filename, left_image_8uc1);

            for (i = 0 ; i < left_image.rows/2 ; i++)
            {
                ib = i*2;
                ptr_i = left_image_8uc1.ptr<uint8_t>(ib);
                ptr_i1 = left_image_8uc1.ptr<uint8_t>(ib+1);
                dbptr = left_image_debayer.ptr<uint8_t>(i);

                rptr_i = right_image_8uc1.ptr<uint8_t>(ib);
                rptr_i1 = right_image_8uc1.ptr<uint8_t>(ib+1);
                rdbptr = right_image_debayer.ptr<uint8_t>(i);

                for ( j = 0 ; j < left_image_8uc1.cols/2 ; j++)
                {
                    jb = j*2;
                    green = int((ptr_i[jb] + ptr_i1[jb+1]))/2;
                    red = int(ptr_i[jb+1]);
                    blue = int(ptr_i1[jb]);
                    // dbptr[j] =  uint8_t((red + red  + blue + green + green + green))/6;
                    dbptr[j] = uint8_t(sqrt(0.299*red*red + 0.587*green*green + 0.114*blue*blue));

                    green = int((rptr_i[jb] + rptr_i1[jb+1]))/2;
                    red = int(rptr_i[jb+1]);
                    blue = int(rptr_i1[jb]);
                    // rdbptr[j] = uint8_t((red + red  + blue + green + green + green))/6;
                    rdbptr[j] = uint8_t(sqrt(0.299*red*red + 0.587*green*green + 0.114*blue*blue));
                }
            }
            t = getTickCount() - t;
            printf("loop debayer time: %fms\n", t * 1000 / getTickFrequency());

            // t = getTickCount();
            // cvtColor(left_image_8uc1, left_image_8uc1, COLOR_BayerGR2BGR);
            // resize(left_image_8uc1, left_image_8uc1, left_image_8uc1.sze(), 0, 0, INTER_AREA);
            // t = getTickCount() - t;
            // printf("Debayer and resize Opencv %fms\n", );

            // rectify
            // set up other values
            image_size = left_image_debayer.size();
            // cout << image_size.height << " " << image_size.width << endl;
            sf = 600. / MAX(image_size.width, image_size.height);
            w = cvRound(image_size.width * sf);
            h = cvRound(image_size.height * sf);

            // undistort and rectify
            t = getTickCount();
            initUndistortRectifyMap(camera_matrix[0], dist_coeffs[0], R1, P1, image_size, CV_16SC2, rmap[0][0], rmap[0][1]);
            initUndistortRectifyMap(camera_matrix[1], dist_coeffs[1], R2, P2, image_size, CV_16SC2, rmap[1][0], rmap[1][1]);
            ROS_INFO("Finish Initializing Undistort Map");
            remap(left_image_debayer, rimg0, rmap[0][0], rmap[0][1], INTER_LINEAR);
            remap(right_image_debayer, rimg1, rmap[1][0], rmap[1][1], INTER_LINEAR);
            ROS_INFO("Finish Remapping");
            // use second region of interest because it is smaller for this specific camera calibration
            Rect vroi(cvRound(VROIX*sf), cvRound(VROIY*sf),
                    cvRound(VROIW*sf), cvRound(VROIH*sf));

            // imwrite("/home/ubuntu/img_log/rim0.jpg", rimg0);
            // imwrite("/home/ubuntu/img_log/rimg1.jpg", rimg1);
            cout << rimg0.rows << " " << rimg0.cols << endl;
            if (rimg0.data)
            {
                cout << "Resizing rimg0" << endl;
                cv::resize(rimg0, rimg0, Size(w, h), 0, 0, INTER_AREA);
            }
            else
                cout << "No rimg0 data" << endl;

            if (rimg1.data)
            {
                cout << "Resizing rimg1" << endl;
                cv::resize(rimg1, rimg1, Size(w, h), 0, 0, INTER_AREA);
            }
            else
                cout << "No rimg1 data" << endl;

            // cout << "Finish resizing image" << endl;
            cropped_left = rimg0(vroi);
            cropped_right = rimg1(vroi);

            sprintf( filename, "/home/ubuntu/img_log/cropped_left%d.jpg", loop_count );
            imwrite(filename, cropped_left);

            sprintf( filename, "/home/ubuntu/img_log/cropped_right%d.jpg", loop_count );
            imwrite(filename, cropped_right);

            SADWindowSize = 3;
            numberOfDisparities = 0;

            sgbm.preFilterCap = 63;
            sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
            // cn = left_image.channels();
            cn = cropped_left.channels();

            image_size = cropped_left.size();
            numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((image_size.width / 8) + 15) & -16;
            sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
            sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
            sgbm.minDisparity = 0;
            sgbm.numberOfDisparities = numberOfDisparities;
            sgbm.uniquenessRatio = 10;
            sgbm.speckleWindowSize = 100;
            sgbm.speckleRange = 32;
            sgbm.disp12MaxDiff = 1;
            sgbm.fullDP = alg == STEREO_HH;

            // sgbm.setMode(StereoSGBM::MODE_SGBM);

            t = getTickCount();
            sgbm(cropped_left, cropped_right, disp);
            t = getTickCount() - t;
            ROS_INFO("Disparity Time elapsed: %fms\n", t * 1000 / getTickFrequency());

            if (alg != STEREO_VAR)
                disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
            else
                disp.convertTo(disp8, CV_8U);
            
            sprintf( filename, "/home/ubuntu/img_log/disp%d.jpg", loop_count );
            imwrite(filename, disp8);
            minMaxLoc(disp8, &min, &max, NULL, NULL);
            
            cout << disp8.rows << " " << disp8.cols << " " << max << endl;
            ellipse_list = calculate_udisparity(disp8, max, image_size, obj_count, loop_count);

            // current_x = local_poses[current_waypoint_index].pose.position.x*100; 
            // current_y = local_poses[current_waypoint_index].pose.position.y*100;

            current_x = 0; 
            current_y = 0;

            GYb = 0; // 0 degree for now
            Mat WRr = (Mat_<double>(2, 2) << cos(90 * M_PI / 180), -sin(90 * M_PI / 180), sin(90 * M_PI / 180), cos(90 * M_PI / 180));
            Mat GRb = (Mat_<double>(2, 2) << cos(GYb * M_PI / 180), -sin(GYb * M_PI / 180), sin(GYb * M_PI / 180), cos(GYb * M_PI / 180));
            Mat GPb = (Mat_<double>(2, 1) << current_x, current_y);
            GPb = WRr * GPb;
            // GPb.ptr<double>(0)[0] = GPb.ptr<double>(0)[0] + 3000; // shift 3000 only for displaying purpose

            obstacle_map.setTo(Scalar(0,0,0));
            for (i = 0; i < obj_count; i++)
            {
                ellipse_list[i].u1 = ellipse_list[i].u1 - image_size.width/2; // set position of the drone at the center of the image
                ellipse_list[i].u2 = ellipse_list[i].u2 - image_size.width/2;
                ellipse_list[i].d1 = ellipse_list[i].d1;
                ellipse_list[i].d2 = ellipse_list[i].d2;
 
                ellipse_list[i].BPe = (Mat_<double>(2, 1) << b*(ellipse_list[i].u1 + ellipse_list[i].u2)/(2*ellipse_list[i].d2), f*b/(ellipse_list[i].d2));
                ellipse_list[i].BSe = (Mat_<double>(2, 1) << b*(ellipse_list[i].u2 - ellipse_list[i].u1)/(2*ellipse_list[i].d2), f*b/(ellipse_list[i].d2) - f*b/(ellipse_list[i].d1));
                ellipse_list[i].GYe = GYb;
                ellipse_list[i].ESe = (Mat_<double>(2, 1) << b*(ellipse_list[i].u2 - ellipse_list[i].u1)/(2*ellipse_list[i].d2), f*b/(ellipse_list[i].d2) - f*b/(ellipse_list[i].d1));
                ellipse_list[i].GPe = GRb * ellipse_list[i].BPe + GPb;
                ellipse_list[i].GSe = GRb * ellipse_list[i].BSe + GPb;
                
                pe1 = ellipse_list[i].GPe.ptr<double>(0);
                pe2 = ellipse_list[i].GPe.ptr<double>(1);
                
                se1 = ellipse_list[i].BSe.ptr<double>(0);
                se2 = ellipse_list[i].BSe.ptr<double>(1);
                se1[0] = se1[0] + 500; // 3 m x boundary
                se2[0] = se2[0] + 1000; // 5 cm y boundary
                
                // cout << "drawn: " << pe1[0] << " " << pe2[0] << " " << 2*se1[0] << " " << 2*se2[0] << endl;
                ellipse(obstacle_map, Point(cvRound(pe1[0] + 3000),cvRound(2000 - pe2[0])), Size(cvRound(se1[0] - 100),cvRound(se2[0] - 100)), 0, 0, 360, Scalar(0,0,255),2);
                ellipse(obstacle_map, Point(cvRound(pe1[0] + 3000),cvRound(2000 - pe2[0])), Size(cvRound(se1[0]),cvRound(se2[0])), 0, 0, 360, Scalar(0,255,0),2);
            }
            num_wp = wp_gen - current_waypoint_index;
            waypoint_checking(local_poses, ellipse_list, obj_count, num_wp, current_waypoint_index);
            
            for ( i = 0 ; i < num_wp - 1 ; i++)
            {
                ROS_INFO("waypoint x: %d y: %d", cvRound(-local_poses[current_waypoint_index+i].pose.position.y*100+3000), cvRound(2000 - local_poses[current_waypoint_index+i].pose.position.x*100));
                // cout << "waypoint" << " " << cvRound(-local_poses[current_waypoint_index+i].pose.position.y*100+3000) << " " << cvRound(2000 - local_poses[current_waypoint_index+i].pose.position.x*100) << " " << endl;
                line(obstacle_map, Point(cvRound(-local_poses[current_waypoint_index+i].pose.position.y*100+3000), cvRound(2000 - local_poses[current_waypoint_index+i].pose.position.x*100)), Point(cvRound(-local_poses[current_waypoint_index+i+1].pose.position.y*100 + 3000), cvRound(2000 - local_poses[current_waypoint_index+i+1].pose.position.x*100)), Scalar(0,0,255), 2);
            }
            for ( i = 0 ; i < num_wp ; i++)
            {
                ROS_INFO("waypoint lat: %.8f long: %.8f", (global_home_pose.latitude - local_poses[current_waypoint_index+i].pose.position.y/111111.0), (global_home_pose.longitude - local_poses[current_waypoint_index+i].pose.position.x/111111.0));
            }

            sprintf( filename, "/home/ubuntu/img_log/obstacle_map%d.jpg", loop_count );
            imwrite(filename, obstacle_map);

            last_calculation = ros::Time::now();
            loop_count++;
            }
        changeGlobalPoseWithRef(global_pose, global_home_pose, local_poses[current_waypoint_index]);
        global_pos_pub.publish(global_pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
