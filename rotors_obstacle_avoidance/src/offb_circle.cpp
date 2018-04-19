#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
#include <cmath>

#include "math.h"

double r;
double theta;
double count=0.0;
double wn;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
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
        std::abs(expectedPosition.pose.position.x - current_pose.pose.position.x) < 0.5 && 
        std::abs(expectedPosition.pose.position.y - current_pose.pose.position.y) < 0.5 && 
        std::abs(expectedPosition.pose.position.z - current_pose.pose.position.z) < 0.5
    );
}

int main(int argc, char **argv)
{
    int i;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    std::vector<geometry_msgs::PoseStamped> poses(20);
    poses[0].pose.position.x = 0;
    poses[0].pose.position.y = 0;
    poses[0].pose.position.z = 5;
    
    // Y -> N
    // X -> E
    for (i = 0 ; i < 20 ; i++)
    {
        if(i <= 5)
        {
            poses[i].pose.position.x = 0;
            poses[i].pose.position.y = i;
            poses[i].pose.position.z = 5;
        }
        else if(i <= 10)
        {
            poses[i].pose.position.x = i - 5;
            poses[i].pose.position.y = 5;
            poses[i].pose.position.z = 5;
        }
        else if(i <= 15)
        {
            poses[i].pose.position.x = 5;
            poses[i].pose.position.y = 15 - i;
            poses[i].pose.position.z = 5;
        }
        else 
        {
            poses[i].pose.position.x = 20 - i;
            poses[i].pose.position.y = 0;
            poses[i].pose.position.z = 5;
        }
    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(poses[0]);
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    i = 0;
    while(ros::ok()) {
        ROS_INFO("Current Mode %s\n", current_state.mode.c_str());
        if (current_state.mode != "GUIDED")
        {
            
            ROS_INFO("Vehicle is not in GUIDED Mode");
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        // if (current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0)))
        // {
        //     if (set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent)
        //     {
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // }
        else
        {
            // std::cout << "Vehicle In Guieded Mode" << std::endl;
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }

        }
        if(checkEqualPose(poses[i%20]))
        {
            i++;
        }
        ROS_INFO("Index: %D Current Pose %.f %.f %.f\n", i,current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z );
        // std::cout << "Index " << "i" << std::endl;
        // std::cout << "Current Pose: " << current_pose.pose.position.x << " " << current_pose.pose.position.y << " " << current_pose.pose.position.z << std::endl;

        local_pos_pub.publish(poses[i%20]);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}