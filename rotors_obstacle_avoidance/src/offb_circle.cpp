#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>

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
    return fabs(current_pose.latitude - 0.0) < std::numeric_limits<double>::epsilon() && fabs(current_pose.longitude - 0.0) < std::numeric_limits<double>::epsilon();
}

bool checkEqualPose(const mavros_msgs::GlobalPositionTarget expectedPosition)
{
    double r = 0.00005;
    double clat = current_pose.latitude;
    double clong = current_post.longitude;
    double elat = expectedPosition.latitude;
    double elong = expectedPosition.longitude;
    ROS_INFO("%.8f %.8f", fabs(clat - elat), fabs(clong - elong));
    // return (current_pose.latitude - expectedPosition.latitude)*(current_pose.latitude - expectedPosition.latitude) + (current_pose.longitude - expectedPosition.longitude)*(current_pose.longitude - expectedPosition.longitude) < r*r;
    return fabs(clat - elat) < r && fabs(clong - elong) < r;
}

void changeGlobalPoseWithRef(mavros_msgs::GlobalPositionTarget &global_pose, mavros_msgs::GlobalPositionTarget &global_home_pose, geometry_msgs::PoseStamped local_pose)
{
    // flat world conversion with 111,111 meter per degree
    global_pose.latitude = global_home_pose.latitude + local_pose.pose.position.x/111111.0; 
    global_pose.longitude = global_home_pose.longitude - local_pose.pose.position.y/111111.0;
    global_pose.altitude = 4.0;
}

int main(int argc, char **argv)
{
    int i;
    int current_waypoint_index;
    int wp_gen = 10;
    bool trigger_new_guided = false;
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

    mavros_msgs::GlobalPositionTarget global_pose, global_home_pose;
    std::vector<geometry_msgs::PoseStamped> local_poses(20);

    global_home_pose.latitude = current_pose.latitude;
    global_home_pose.longitude = current_pose.longitude;
    global_home_pose.altitude = 4.0;
    global_home_pose.velocity.x = 2.0;
    global_home_pose.velocity.y = 2.0;
    global_home_pose.velocity.z = 2.0;
    global_home_pose.acceleration_or_force.x = 2;
    global_home_pose.acceleration_or_force.y = 2;
    global_home_pose.acceleration_or_force.z = 2;
    global_home_pose.coordinate_frame = gbpos::FRAME_GLOBAL_REL_ALT;
    global_home_pose.type_mask = 4088;

    global_pose.latitude = current_pose.latitude + 0.005;
    global_pose.longitude = current_pose.longitude;
    global_pose.altitude = 4.0;
    global_pose.velocity.x = 2.0;
    global_pose.velocity.y = 2.0;
    global_pose.velocity.z = 2.0;
    global_pose.acceleration_or_force.x = 2;
    global_pose.acceleration_or_force.y = 2;
    global_pose.acceleration_or_force.z = 2;
    global_pose.coordinate_frame = gbpos::FRAME_GLOBAL_REL_ALT;
    global_pose.type_mask = 4088;

    // mock up waypoint
    local_poses[0].pose.position.x = 0.0; // in meter
    local_poses[0].pose.position.y = 0.0;
    local_poses[0].pose.position.z = 4.0;

    local_poses[1].pose.position.x = 10.0; // in meter
    local_poses[1].pose.position.y = 0.0;
    local_poses[1].pose.position.z = 4.0;

    local_poses[2].pose.position.x = 10.0; // in meter
    local_poses[2].pose.position.y = -10.0;
    local_poses[2].pose.position.z = 4.0;

    local_poses[3].pose.position.x = 20.0; // in meter
    local_poses[3].pose.position.y = -10.0;
    local_poses[3].pose.position.z = 4.0;

    local_poses[4].pose.position.x = 20.0; // in meter
    local_poses[4].pose.position.y = 0.0;
    local_poses[4].pose.position.z = 4.0;

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
            trigger_new_guided = false;
            if (!trigger_new_guided)
            {
                // if re-trigger guided mode, set new global zero position
                global_home_pose.latitude = current_pose.latitude;
                global_home_pose.longitude = current_pose.longitude;
                // global_home_pose.altitude = current_pose.altitude;

                global_pose.latitude = global_home_pose.latitude;
                global_pose.longitude = global_home_pose.longitude;
                // global_pose.altitude = global_home_pose.altitude;

                current_waypoint_index = 0;
                trigger_new_guided = true;
            }
        }

        if(checkEqualPose(global_pose))
        {
            current_waypoint_index++;
            current_waypoint_index %= 5;
        }

        changeGlobalPoseWithRef(global_pose, global_home_pose, local_poses[current_waypoint_index]);
        global_pos_pub.publish(global_pose);
        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}