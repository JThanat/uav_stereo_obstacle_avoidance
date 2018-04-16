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
#include <vector>
#include <cmath>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "umap_util.hpp"
#include "wp_planning.hpp"

using namespace std;
using namespace umap_utility;
using namespace wp;

cv::Mat left_image;
bool has_left_image = false;
void image_cb_left(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge:: Exception& e)
    {
        ROS_ERROR("Count not convert from '%s' to 'bgr8'. ", msg->encoding.c_str());
    }
    // ROS_INFO("READING IMAGE");
    left_image = cv_ptr->image;
    has_left_image = true;
}

cv::Mat right_image;
bool has_right_image = false;
void image_cb_right(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge:: Exception& e)
    {
        ROS_ERROR("Count not convert from '%s' to 'bgr8'. ", msg->encoding.c_str());
    }
    // ROS_INFO("READING IMAGE");
    right_image = cv_ptr->image;
    has_right_image = true;
}


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
    cv::Size image_size;
    
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
    
    cv::Mat disp(600,800, CV_8UC3);
    cv::Mat disp8(600,800, CV_8UC3);
    cv::Mat obstacle_map(2000,6000, CV_8UC3);

    double GYb;
    double *pe1, *pe2, *se1, *se2;

    int alg = STEREO_SGBM;
    Ptr<StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_left_sub = it.subscribe("/iris/camera_left/image_raw",10,image_cb_left);
    image_transport::Subscriber img_right_sub = it.subscribe("/iris/camera_right/image_raw",10,image_cb_right);
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
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_calculation;
    
    f = 30; // in pixel 1 millimeter = 3.779528 pixel
    b = 15;   // in cm

    loop_count = 0;
    last_calculation = ros::Time::now();
    // if(current_state.mode == "OFFBOARD") enabled = true;
    // if(current_state.armed) armed = true;
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
                enabled = true;
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (current_state.mode == "OFFBOARD") {
                enabled = true;
            }
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                    armed = true;
                }
                last_request = ros::Time::now();
            }
        }

        if (!enabled || !armed) {
            local_pos_pub.publish(poses[0]);
            ros::spinOnce();
        }

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
            if(!enabled || !armed) continue;
            if (!has_left_image || !has_right_image) continue;
            image_size = left_image.size();

            SADWindowSize = 3;
            numberOfDisparities = 0;

            sgbm->setPreFilterCap(63);
            sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
            sgbm->setBlockSize(sgbmWinSize);
            cn = left_image.channels();
            // cn = cropped_left.channels();

            // image_size = cropped_left.size();
            numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((image_size.width / 8) + 15) & -16;
            sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);
            sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);
            sgbm->setMinDisparity(0);
            sgbm->setNumDisparities(numberOfDisparities);
            sgbm->setUniquenessRatio(10);
            sgbm->setSpeckleWindowSize(100);
            sgbm->setSpeckleRange(32);
            sgbm->setDisp12MaxDiff(1);

            sgbm->setMode(StereoSGBM::MODE_SGBM);

            t = getTickCount();
            sgbm->compute(left_image, right_image, disp);
            t = getTickCount() - t;
            printf("Disparity Time elapsed: %fms\n", t * 1000 / getTickFrequency());

            if (alg != STEREO_VAR)
                disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
            else
                disp.convertTo(disp8, CV_8U);
            
            sprintf( filename, "./disp%d.jpg", loop_count );
            imwrite(filename, disp8);
            cv::minMaxLoc(disp8, &min, &max, NULL, NULL);
            
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

            obstacle_map.setTo(cv::Scalar(0,0,0));
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
            sprintf( filename, "./obstacle_map%d.jpg", loop_count );
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
