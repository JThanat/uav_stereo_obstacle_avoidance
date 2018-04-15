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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

Mat disp;
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
    disp = cv_ptr->img;
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
    std::vector<geometry_msgs::PoseStamped> avoidingPath(3);
    int i,j,k;
    int avoidingIdx = 0;
    int avoidingSize = 3;
    bool armed = false;
    bool enabled = false;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber disp_sub = it.subscribe("/stereo_msgs/DisparityImage",1,image_cb);
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

        if(avoiding) {
            // avoiding in guided mode
            ROS_INFO("Avoiding");
            if(avoidingIdx == avoidingSize) {
                avoiding = false;
                continue;
            }
            if(checkEqualPose(avoidingPath[avoidingIdx])) {
                avoidingIdx++;
            }
            local_pos_pub.publish(avoidingPath[avoidingIdx]);
            ros::spinOnce();
            
        } else {
            // keep moving to goal
            local_pos_pub.publish(poses[1]);
            ros::spinOnce();
        }
        
        // if(checkEqualPose(poses[i%4])) {
        //     i++;
        //     ROS_INFO("i: %d",i);
        //     local_pos_pub.publish(poses[i%4]);
        //     ros::spinOnce();
        // }

        // local_pos_pub.publish(poses[i%4]);
        // ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
