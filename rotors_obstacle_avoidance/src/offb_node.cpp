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
    dangerPose.pose.position.x = 0;
    dangerPose.pose.position.y = 10;
    dangerPose.pose.position.z = 2;

    if(dangerPose.pose.position.y - current_pose.pose.position.y < 4 && 
    dangerPose.pose.position.y - current_pose.pose.position.y > 0 && !avoiding) {
        avoiding = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    std::vector<geometry_msgs::PoseStamped> poses(4);
    std::vector<geometry_msgs::PoseStamped> avoidingPath(3);
    int i = 0;
    int avoidingIdx = 0;
    int avoidingSize = 3;
    bool armed = false;
    bool enabled = false;

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

    // initial
    poses[0].pose.position.x = 0;
    poses[0].pose.position.y = 0;
    poses[0].pose.position.z = 2;

    // goal
    poses[1].pose.position.x = 0;
    poses[1].pose.position.y = 25;
    poses[1].pose.position.z = 2;

    // poses[2].pose.position.x = 5;
    // poses[2].pose.position.y = 5;
    // poses[2].pose.position.z = 2;

    // poses[3].pose.position.x = 5;
    // poses[3].pose.position.y = 0;
    // poses[3].pose.position.z = 2;

    avoidingPath[0].pose.position.x = 5;
    avoidingPath[0].pose.position.y = 10;
    avoidingPath[0].pose.position.z = 2;

    avoidingPath[1].pose.position.x = 5;
    avoidingPath[1].pose.position.y = 15;
    avoidingPath[1].pose.position.z = 2;

    avoidingPath[2].pose.position.x = 2;
    avoidingPath[2].pose.position.y = 15;
    avoidingPath[2].pose.position.z = 2;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(poses[0]);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("OK wit setpoints")

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        ROS_INFO("Setting up things");
        if (current_state.mode == "AUTO" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            offb_set_mode.request.custom_mode = "GUIDED";
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success)
            {
                ROS_INFO("GUIDED enabled");
            }
            last_request = ros::Time::now();
        }

        if (current_state.mode == "GUIDED" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {   
            ROS_INFO("GUIDED MODE");
        }

        if (current_state.mode != "GUIDED" && current_state.mode != "AUTO" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            ROS_INFO("UNKNOWN MODE");
        }
        rate.sleep();
    }

    return 0;
}
