#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <cmath>

#include "math.h"

double r;
double theta;
double count=0.0;
double wn;

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

bool atZeroLatLong()
{
    // check if it is around 0 0 point
    return abs(current_pose.latitude - 0.0) < std::numeric_limits<double>::epsilon() && abs(current_pose.longitude - 0.0) < std::numeric_limits<double>::epsilon();
}

// bool checkEqualPose(const geometry_msgs::PoseStamped expectedPosition)
// {
//     return (
//         std::abs(expectedPosition.pose.position.x - current_pose.pose.position.x) < 0.3 && 
//         std::abs(expectedPosition.pose.position.y - current_pose.pose.position.y) < 0.3 && 
//         std::abs(expectedPosition.pose.position.z - current_pose.pose.position.z) < 0.3
//     );
// }

int main(int argc, char **argv)
{
    int i;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, pose_cb);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        // std::cout << current_pose.latitude << " " << current_pose.longitude << std::endl;
        ros::spinOnce();
        rate.sleep();
    }

    while(atZeroLatLong())
    {
        // std::cout << current_pose.latitude << " " << current_pose.longitude << std::endl;
        ROS_INFO("check at zero point %.6f %.6f %.6f", current_pose.latitude, current_pose.longitude, current_pose.altitude);
        ros::spinOnce();
        rate.sleep();
    }

    std::vector<mavros_msgs::GlobalPositionTarget> poses(20);
    geometry_msgs::PoseStamped local_pose;
    local_pose.pose.position.x = 0;
    local_pose.pose.position.y = 0;
    local_pose.pose.position.z = 5;

    poses[0].latitude = current_pose.latitude;
    poses[0].longitude = current_pose.longitude;
    poses[0].altitude = 5;
    poses[0].velocity.x = 2.0;
    poses[0].velocity.y = 2.0;
    poses[0].velocity.z = 2.0;
    poses[0].acceleration_or_force.x = 2;
    poses[0].acceleration_or_force.y = 2;
    poses[0].acceleration_or_force.z = 2;
    poses[0].coordinate_frame = gbpos::FRAME_GLOBAL_REL_ALT;
    poses[0].type_mask = 4088;

    std::cout <<  poses[0].latitude << " " <<  poses[0].longitude << " " << poses[0].altitude << std::endl;

    poses[1].latitude = current_pose.latitude + 0.01;
    poses[1].longitude = current_pose.longitude;
    poses[1].altitude = 5;
    poses[1].velocity.x = 2.0;
    poses[1].velocity.y = 2.0;
    poses[1].velocity.z = 2.0;
    poses[1].acceleration_or_force.x = 2;
    poses[1].acceleration_or_force.y = 2;
    poses[1].acceleration_or_force.z = 2;
    poses[1].coordinate_frame = gbpos::FRAME_GLOBAL_REL_ALT;
    poses[1].type_mask = 4088;
    

    // std::cout <<  poses[1].latitude << " " <<  poses[1].longitude << " " << poses[1].altitude << std::endl;
    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        global_pos_pub.publish(poses[0]);
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    bool enabled = false;
    bool armed = false;

    ros::Time last_request = ros::Time::now();

    i = 0;
    while(ros::ok()) {
        if (current_state.mode != "GUIDED") {
            ROS_INFO("Vehicle is not in GUIDED Mode");
            ros::spinOnce();
            rate.sleep();
            continue;
        } else {
            ROS_INFO("Pub 1");
            ROS_INFO("PUB TO %.6f %.6f %.6f", poses[1].latitude, poses[1].longitude, poses[1].altitude);
            global_pos_pub.publish(poses[1]);
            ros::spinOnce();
            rate.sleep();
        }

    }

    return 0;
}