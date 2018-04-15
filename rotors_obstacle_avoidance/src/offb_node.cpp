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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "umap_util.hpp"
#include "wp_planning.hpp"

cv::Mat disp;
void image_cb(const sensor_msgs::ImageConstPtr& msg)
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
    ROS_INFO("READING IMAGE");
    disp = cv_ptr -> image;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void isInCrashingZone();

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    isInCrashingZone();
}

bool checkEqualPose(const geometry_msgs::PoseStamped expectedPosition)
{
    return (
        std::abs(expectedPosition.pose.position.x - current_pose.pose.position.x) < 0.5 && 
        std::abs(expectedPosition.pose.position.y - current_pose.pose.position.y) < 0.5 && 
        std::abs(expectedPosition.pose.position.z - current_pose.pose.position.z) < 0.5
    );
}

bool avoiding = false;
void isInCrashingZone() {
    // check if in crashing zone 
    // hardcoded for now
    geometry_msgs::PoseStamped dangerPose;
    dangerPose.pose.position.x = 10;
    dangerPose.pose.position.y = 0;
    dangerPose.pose.position.z = 2;

    if(dangerPose.pose.position.x - current_pose.pose.position.x < 4 && 
    dangerPose.pose.position.x - current_pose.pose.position.x > 0 && !avoiding) {
        avoiding = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    std::vector<geometry_msgs::PoseStamped> poses(200);

    double min, max;
    int i,j,k,current_x, current_y;
    int avoidingIdx = 0;
    int avoidingSize = 3;
    int current_waypoint_index = 0;
    int num_wp = 0;
    bool armed = false;
    bool enabled = false;
    vector<ellipse_desc> ellipse_list;
    vector< pair<double, double> > waypoints(200);
    vector< pair<double, double> > waypoints_pub(200);
    cv::Mat disp8(600,800, CV_8UC3);


    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/iris/camera_left/image_raw",10,image_cb);
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
    for (i = 0; i < 200 ; i++)
    {
        poses[i].pose.position.x = 0.1*i;
        poses[i].pose.position.y = 0.0;
        poses[i].pose.position.z = 5.0;
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
    
    loop_count = 0;
    last_calculation = ros::Time::now();
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

        if(checkEqualPose(poses[current_waypoint_index]))
        {
            current_waypoint_index++;
        }

        if(loop_count == 0 || ros::Time::now() - last_calculation > 0.5)
        {
            image_size = disp.size();
            disp.convertTo(disp8, CV_8U);
            minMaxLoc(disp8, &min, &max, NULL, NULL);

            ellipse_list = calculate_udisparity(disp8, max, image_size, obj_count);

            current_x = poses[current_waypoint_index].pose.position.x; // need update
            current_y = poses[current_waypoint_index].pose.position.y; // need update

            GYb = 0; // 0 degree for now
            Mat GRb = (Mat_<double>(2, 2) << cos(GYb * M_PI / 180), -sin(GYb * M_PI / 180), sin(GYb * M_PI / 180), cos(GYb * M_PI / 180));
            Mat GPb = (Mat_<double>(2, 1) << current_x, current_y);
            
            for (i = 0; i < obj_count; ++i)
            {
                ellipse_list[i].u1 = ellipse_list[i].u1; // set position of the drone at the center of the image
                ellipse_list[i].u2 = ellipse_list[i].u2;
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
                se1[0] = se1[0] + 50; // 50 cm boundary
                se2[0] = se2[0] + 50; // 50 cm boundary
            }
            num_wp = 200 - currnet_index;
            for ( i = 0; i < num_wp ; i++)
            {
                waypoint[i] = make_pair(poses[current_waypoint_index + i].pose.position.x, poses[current_waypoint_index].pose.position.y);
            }
            waypoint_checking(waypoints, waypoints_pub, ellipse_list, obj_count, num_wp);

            }

            last_calculation = ros::Time::now();
            loop_count++;
        }

        rate.sleep();
    }

    return 0;
}
