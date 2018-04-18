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

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
}

bool checkEqualPose(const geometry_msgs::PoseStamped expectedPosition)
{
    return (
        std::abs(expectedPosition.pose.position.x - current_pose.pose.position.x) < 0.1 && 
        std::abs(expectedPosition.pose.position.y - current_pose.pose.position.y) < 0.1 && 
        std::abs(expectedPosition.pose.position.z - current_pose.pose.position.z) < 0.1
    );
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
    std::vector<geometry_msgs::PoseStamped> poses(200);

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
    int wp_gen = 200;
    vector<ellipse_desc> ellipse_list(20);
    vector< pair<double, double> > waypoints(200);
    vector< pair<double, double> > waypoints_pub(200);
    
    int framesize = image_size.width*image_size.height;

    Mat left_image(1842, 2432, CV_16UC1);
    Mat right_image(1842, 2432, CV_16UC1);
    Mat left_image_debayer(921, 1216, CV_8UC1);
    Mat right_image_debayer(921, 1216, CV_8UC1);
    Mat disp(600,800, CV_8UC3);
    Mat disp8(600,800, CV_8UC3);
    Mat obstacle_map(2000,6000, CV_8UC3);

    Mat R, T, R1, R2, P1, P2, Q;
    Mat cropped_left, cropped_right;
    Mat camera_matrix[2], dist_coeffs[2];
    Mat rimg[2], cimg;
    Mat rmap[2][2];
    Rect validRoi[2];
    int VROIX, VROIY, VROIW, VROIH;
    int sf, w, h;

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
        cout << "Error: can not read the intrinsic parameters\n";

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
        cout << "Error: can not read the extrinsic parameters\n";

	
    std::thread spi_thread(getimage::flushBuffer); // signal every 250 ms
	cameraState *c1 = init_camera(dev_name, 2432, 1842, 1, 3, 2);
	cameraState *c2 = init_camera(dev_name2, 2432, 1842, 1, 3, 2);

	int control_val, control_val2;
	v4l2SetControl(c1, V4L2_CID_EXPOSURE, 10);
	control_val = v4l2GetControl(c1, V4L2_CID_EXPOSURE);
	fprintf(stderr, "set value:%d\n", control_val);

	v4l2SetControl(c1, V4L2_CID_GAIN, 1);
	control_val = v4l2GetControl(c1, V4L2_CID_GAIN);
	fprintf(stderr, "set value:%d\n", control_val);

	v4l2SetControl(c2, V4L2_CID_EXPOSURE, 10);
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
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ROS_INFO("current state connected %d", current_state.connected);
        ros::spinOnce();
        ROS_INFO("SPIN!");
        rate.sleep();
        ROS_INFO("Wake up");
    }
    ROS_INFO("set pose array");

    // mock up waypoint
    for (i = 0; i < wp_gen ; i++)
    {
        poses[i].pose.position.x = 0.2*i;
        poses[i].pose.position.y = 0.0;
        poses[i].pose.position.z = 2.0;
    }

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(poses[0]);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_calculation;
    
    f = 30.236220472; // in pixel 1 millimeter = 3.779528 pixel
    b = 15;   // in cm

    loop_count = 0;
    last_calculation = ros::Time::now();
    // if(current_state.mode == "OFFBOARD") enabled = true;
    // if(current_state.armed) armed = true;
    while (ros::ok())
    {
        if (current_state.mode != "GUIDED")
        {   
            ROS_INFO("Not in GUIDED MODE");
            enabled = false;
            rate.sleep();
            continue;
        }
        else
        {
            enabled = true;
        }

        if (!enabled) {
            local_pos_pub.publish(poses[0]);
            ros::spinOnce();
        }
        ready = true; // TODO REMOVE THIS!
        if (!ready && checkEqualPose(poses[0]))
        {
            ready = true;
        }

        if(checkEqualPose(poses[current_waypoint_index]) && current_waypoint_index != 200)
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
            left_image.convertTo(left_image, CV_8UC1, 1);
            right_image.convertTo(right_image, CV_8UC1, 1);
            // imwrite("/home/ubuntu/img_log/left_image.jpg", left_image);
            for (i = 0 ; i < left_image.rows/2 ; i++)
            {
                ib = i*2;
                ptr_i = left_image.ptr<uint8_t>(ib);
                ptr_i1 = left_image.ptr<uint8_t>(ib+1);
                dbptr = left_image_debayer.ptr<uint8_t>(i);

                rptr_i = right_image.ptr<uint8_t>(ib);
                rptr_i1 = right_image.ptr<uint8_t>(ib+1);
                rdbptr = right_image_debayer.ptr<uint8_t>(i);

                for ( j = 0 ; j < left_image.cols/2 ; j++)
                {
                    jb = j*2;
                    green = int((ptr_i[jb] + ptr_i1[jb+1])/2);
                    red = int(ptr_i[jb+1]);
                    blue = int(ptr_i1[jb]);
                    dbptr[j] = uint8_t(sqrt(0.299*red*red + 0.587*green*green + 0.114*blue*blue));

                    green = int(rptr_i[jb] + rptr_i1[jb+1])/2;
                    red = int(rptr_i[jb+1]);
                    blue = int(rptr_i1[jb]);
                    rdbptr[j] = uint8_t(sqrt(0.299*red*red + 0.587*green*green + 0.114*blue*blue));
                }
            }
            t = getTickCount() - t;
            printf("loop debayer time: %fms\n", t * 1000 / getTickFrequency());
            // imwrite("/home/ubuntu/img_log/left_debayer.jpg", left_image_debayer);

            // rectify
            // set up other values
            image_size = left_image_debayer.size();
            sf = 600. / MAX(image_size.width, image_size.height);
            w = cvRound(image_size.width * sf);
            h = cvRound(image_size.height * sf);

            // undistort and rectify
            t = getTickCount();
            initUndistortRectifyMap(camera_matrix[0], dist_coeffs[0], R1, P1, image_size, CV_16SC2, rmap[0][0], rmap[0][1]);
            initUndistortRectifyMap(camera_matrix[1], dist_coeffs[1], R2, P2, image_size, CV_16SC2, rmap[1][0], rmap[1][1]);

            remap(left_image_debayer, rimg[0], rmap[0][0], rmap[0][1], INTER_LINEAR);
            remap(right_image_debayer, rimg[1], rmap[1][0], rmap[1][1], INTER_LINEAR);

            // use second region of interest because it is smaller for this specific camera calibration
            Rect vroi(cvRound(VROIX * sf), cvRound(VROIY * sf),
                    cvRound(VROIW * sf), cvRound(VROIH * sf));
            cout << rimg[0].rows << " " << rimg[0].cols << endl;
            resize(rimg[0], rimg[0], Size(w, h), 0, 0, INTER_AREA);
            resize(rimg[1], rimg[1], Size(w, h), 0, 0, INTER_AREA);
            cropped_left = rimg[0](vroi);
            cropped_right = rimg[1](vroi);

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
            printf("Disparity Time elapsed: %fms\n", t * 1000 / getTickFrequency());

            if (alg != STEREO_VAR)
                disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
            else
                disp.convertTo(disp8, CV_8U);
            
            sprintf( filename, "/home/ubuntu/img_log/disp%d.jpg", loop_count );
            imwrite(filename, disp8);
            minMaxLoc(disp8, &min, &max, NULL, NULL);
            
            // for mock up only
            if (loop_count == 1)
            {
                ROS_INFO("Mock up map");
                image_size = Size(800,600);
                obj_count = 1;
                ellipse_list[0].u1 = 300;
                ellipse_list[0].u2 = 500;
                ellipse_list[0].d1 = 16.0;
                ellipse_list[0].d2 = 14.4;
            } else {
                ellipse_list = calculate_udisparity(disp8, max, image_size, obj_count, loop_count);
            }

            current_x = poses[current_waypoint_index].pose.position.x*100; // need update
            current_y = poses[current_waypoint_index].pose.position.y*100; // need update

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
                ellipse_list[i].d1 = ellipse_list[i].d1/16;
                ellipse_list[i].d2 = ellipse_list[i].d2/16;
 
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
                se1[0] = se1[0] + 100; // 50 cm x boundary
                se2[0] = se2[0] + 100; // 100 cm y boundary
                
                // cout << "drawn: " << pe1[0] << " " << pe2[0] << " " << 2*se1[0] << " " << 2*se2[0] << endl;
                ellipse(obstacle_map, Point(cvRound(pe1[0] + 3000),cvRound(2000 - pe2[0])), Size(cvRound(se1[0] - 50),cvRound(se2[0] - 50)), 0, 0, 360, Scalar(0,0,255),2);
                ellipse(obstacle_map, Point(cvRound(pe1[0] + 3000),cvRound(2000 - pe2[0])), Size(cvRound(se1[0]),cvRound(se2[0])), 0, 0, 360, Scalar(0,255,0),2);
            }
            num_wp = wp_gen - current_waypoint_index;
            waypoint_checking(poses, ellipse_list, obj_count, num_wp, current_waypoint_index);
            
            for ( i = 0 ; i < num_wp - 1 ; i++)
            {
                cout << "waypoint" << " " << cvRound(-poses[current_waypoint_index+i].pose.position.y*100+3000) << " " << cvRound(2000 - poses[current_waypoint_index+i].pose.position.x*100) << " " << endl;
                line(obstacle_map, Point(cvRound(-poses[current_waypoint_index+i].pose.position.y*100+3000), cvRound(2000 - poses[current_waypoint_index+i].pose.position.x*100)), Point(cvRound(-poses[current_waypoint_index+i+1].pose.position.y*100 + 3000), cvRound(2000 - poses[current_waypoint_index+i+1].pose.position.x*100)), Scalar(0,0,255), 2);
            }
            sprintf( filename, "/home/ubuntu/img_log/obstacle_map%d.jpg", loop_count );
            imwrite(filename, obstacle_map);

            last_calculation = ros::Time::now();
            loop_count++;
            }
        local_pos_pub.publish(poses[current_waypoint_index]);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
