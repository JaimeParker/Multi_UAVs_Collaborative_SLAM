//
// Created by hazyparker on 22-5-20.
// control ccm 2 UAVs in gazebo

// include headers
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Waypoint.h>

#include <iostream>
#include <cmath>
#include <termios.h>
#include <cstdio>

#include <geometry_msgs/Twist.h>

// define Key Values
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_T 0x74
#define KEYCODE_L 0x6c
#define KEYCODE_H 0x68

// define speed value
#define MAX_SPEED 1
#define MIN_SPEED -1
#define ACCEL 0.02

using namespace std;

mavros_msgs::State current_state; /* NOLINT */
mavros_msgs::State current_state1; /* NOLINT */
geometry_msgs::PoseStamped current_pose; /* NOLINT */
geometry_msgs::PoseStamped current_pose1; /* NOLINT */

// callback function for current state
void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}
void state_cb1(const mavros_msgs::State::ConstPtr &msg){
    current_state1 = *msg;
}

// callback function for Subscriber for local_pos_sub
void local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}
void local_cb1(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose1 = *msg;
}

// check if reached a waypoint
bool check_waypoint(const geometry_msgs::PoseStamped &now_pose, const geometry_msgs::PoseStamped &aim_pose){
    // define Point to hold current position and aim position
    geometry_msgs::Point curr, aim;
    curr = now_pose.pose.position;
    aim = aim_pose.pose.position;
    double precision = 0.1;

    // define return value
    bool reach = false;

    // calculate distance
    double dist = sqrt(pow((curr.x - aim.x), 2) +
                       pow((curr.y - aim.y), 2) + pow((curr.z - aim.z), 2));
    if(dist < precision){
        reach = true;
        ROS_INFO("reached waypoint!");
    }

    return reach;
}

// proceed landing process
void land(ros::ServiceClient& client0, ros::ServiceClient& client1){
    // set set-point publish rate, MUST more than 2Hz
    ros::Rate rate(10.0);

    // record current time
    ros::Time last_request = ros::Time::now();

    // proceed landing process
    ROS_INFO("landing...");
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";
    while (ros::ok()){
        if( current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            client0.call(land_set_mode);
            client1.call(land_set_mode);
            if(land_set_mode.response.mode_sent) ROS_INFO("Land enabled");
            last_request = ros::Time::now();
        }
        if(!current_state.armed){
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

}

// change to offboard mode and arm
void Offboard(ros::ServiceClient &set_mode_client, mavros_msgs::SetMode &offb_set_mode,
              mavros_msgs::CommandBool &arm_cmd, ros::ServiceClient &arming_client,
              ros::Publisher &local_pos_pub, geometry_msgs::PoseStamped &pose,
              ros::Rate &rate) {
    ros::Time last_request = ros::Time::now();
    ROS_INFO("Off boarding");

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Off-board mode enabling...");
            }
            last_request = ros::Time::now();

        } else {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (current_state.mode != "OFFBOARD") ROS_INFO("Off board mode was shut unexpectedly");
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        //  wait until reach set point
        ros::spinOnce();
        // define Point: current position and set point position (expected)
        geometry_msgs::Point curr,aim;
        curr = current_pose.pose.position;
        aim = pose.pose.position;
        double dist = sqrt(pow((curr.x - aim.x), 2) +
                           pow((curr.y - aim.y), 2) + pow((curr.z - aim.z), 2));
        if(dist < 0.1){
            ROS_INFO("reached the goal...");
            break;
        }
        rate.sleep();

    }

    // hover for a while
    // hover at set point(pose)
    for(int i = 20; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Control Back to user!");
}

int kfd = 0;
struct termios cooked, raw;

// kill this node
void quit(int sig){
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv){
    // orders respectively for UAVs
    // ros node, node handle, client, publisher and subscriber
    // init ros node
    ros::init(argc, argv, "wayPoint_node");

    // UAV 0, leader
    // create node handle
    ros::NodeHandle n0;
    // define subscriber and publisher
    ros::Subscriber state_sub0 = n0.subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub0 = n0.advertise<geometry_msgs::PoseStamped>
            ("uav0/mavros/setpoint_position/local", 10);
    ros::Subscriber local_pos_sub0 = n0.subscribe<geometry_msgs::PoseStamped>
            ("uav0/mavros/local_position/pose", 10, local_cb);
    // define client and server
    ros::ServiceClient arming_client0 = n0.serviceClient<mavros_msgs::CommandBool>
            ("uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client0 = n0.serviceClient<mavros_msgs::SetMode>
            ("uav0/mavros/set_mode");
    ros::Publisher cmd_vel_pub0 = n0.advertise<geometry_msgs::Twist>
            ("uav0/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    // UAV 1
    // create node handle
    ros::NodeHandle n1;
    // define subscriber and publisher
    ros::Subscriber state_sub1 = n1.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, state_cb1);
    ros::Publisher local_pos_pub1 = n1.advertise<geometry_msgs::PoseStamped>
            ("uav1/mavros/setpoint_position/local", 10);
    ros::Subscriber local_pos_sub1 = n1.subscribe<geometry_msgs::PoseStamped>
            ("uav1/mavros/local_position/pose", 10, local_cb1);
    // define client and server
    ros::ServiceClient arming_client1 = n1.serviceClient<mavros_msgs::CommandBool>
            ("uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client1 = n1.serviceClient<mavros_msgs::SetMode>
            ("uav1/mavros/set_mode");
    ros::Publisher cmd_vel_pub1 = n1.advertise<geometry_msgs::Twist>
            ("uav1/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    // set set-point publish rate, MUST more than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("UAV0: connecting to fcu...");
    }
    ROS_INFO("UAV0: fcu connected successfully!");
    while(ros::ok() && !current_state1.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("UAV1: connecting to fcu...");
    }
    ROS_INFO("UAV1: fcu connected successfully!");

    // set msgs for SetMode
    mavros_msgs::SetMode offboard_mode_set;
    mavros_msgs::SetMode offboard_mode_set1;
    offboard_mode_set.request.custom_mode = "OFFBOARD";
    offboard_mode_set1.request.custom_mode = "OFFBOARD";

    // set msgs for arming
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandBool arm_cmd1;
    arm_cmd.request.value = true;
    arm_cmd1.request.value = true;

    // set first waypoint
    geometry_msgs::PoseStamped pose_uav0;
    pose_uav0.pose.position.x = 0;
    pose_uav0.pose.position.y = 0;
    pose_uav0.pose.position.z = 3;
    geometry_msgs::PoseStamped pose_uav1;
    pose_uav1.pose.position.x = 1;
    pose_uav1.pose.position.y = 0;
    pose_uav1.pose.position.z = 3;

    //send a few set-points before starting
    for(int i = 200; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(pose_uav0);
        local_pos_pub1.publish(pose_uav1);
        ros::spinOnce();
        rate.sleep();
    }

    char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("KeyBoard connected! Reading from keyboard...");
    puts("Instructions:");
    puts("---------------------------");
    puts("Press T to shift to offboard mode and take off for UAV0");
    puts("Press L to shift to offboard mode and take off for UAV1");
    puts("Press H to hold position");

    // define Twist velocity
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    //twist.linear.z = 0;
    geometry_msgs::Twist twist1;
    twist1.linear.x = 0;
    twist1.linear.y = 0;
    //twist1.linear.z = 0;

    bool flag = false;

    for(;;){
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0){
            perror("read():");
            exit(-1);
        }

        switch (c) {
            case KEYCODE_T:
                ROS_INFO("Offboard for UAV0");
                Offboard(set_mode_client0, offboard_mode_set, arm_cmd,
                         arming_client0, local_pos_pub0, pose_uav0, rate);
                break;
            case KEYCODE_L:
                ROS_INFO("Offboard for UAV1");
                Offboard(set_mode_client1, offboard_mode_set1, arm_cmd1,
                         arming_client1, local_pos_pub1, pose_uav1, rate);
                break;
            case KEYCODE_H:
                ROS_INFO("Hold position and speed");
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist1.linear.x = 0;
                twist1.linear.y = 0;

            case KEYCODE_W:
                twist.linear.x += ACCEL;
                if (twist.linear.x > MAX_SPEED) twist.linear.x = MAX_SPEED;
                ROS_INFO("add X Speed for UAV0: %f", twist.linear.x);
                flag  = true;
                break;
            case KEYCODE_S:
                twist.linear.x -= ACCEL;
                if (twist.linear.x < MIN_SPEED) twist.linear.x = MIN_SPEED;
                ROS_INFO("dec X Speed for UAV0: %f", twist.linear.x);
                flag  = true;
                break;
            case KEYCODE_A:
                twist.linear.y += ACCEL;
                if (twist.linear.y > MAX_SPEED) twist.linear.y = MAX_SPEED;
                ROS_INFO("add Y Speed for UAV0: %f", twist.linear.y);
                flag  = true;
                break;
            case KEYCODE_D:
                twist.linear.y -= ACCEL;
                if (twist.linear.y < MIN_SPEED) twist.linear.y = MIN_SPEED;
                ROS_INFO("dec Y Speed for UAV0: %f", twist.linear.y);
                flag  = true;
                break;

            case KEYCODE_UP:
                twist1.linear.x += ACCEL;
                if (twist1.linear.x > MAX_SPEED) twist1.linear.x = MAX_SPEED;
                ROS_INFO("add X Speed for UAV1: %f", twist1.linear.x);
                flag  = true;
                break;
            case KEYCODE_DOWN:
                twist1.linear.x -= ACCEL;
                if (twist1.linear.x < MIN_SPEED) twist1.linear.x = MIN_SPEED;
                ROS_INFO("dec X Speed for UAV1: %f", twist1.linear.x);
                flag  = true;
                break;
            case KEYCODE_LEFT:
                twist1.linear.y += ACCEL;
                if (twist1.linear.y > MAX_SPEED) twist1.linear.y = MAX_SPEED;
                ROS_INFO("add Y Speed for UAV1: %f", twist1.linear.y);
                flag  = true;
                break;
            case KEYCODE_RIGHT:
                twist1.linear.y -= ACCEL;
                if (twist1.linear.y < MIN_SPEED) twist1.linear.y = MIN_SPEED;
                ROS_INFO("dec Y Speed for UAV1: %f", twist1.linear.y);
                flag  = true;
                break;

            case KEYCODE_Q:
                ROS_DEBUG("quit");
                return 0;

        }

        for (int i = 0; i < 100; i++){
            cmd_vel_pub0.publish(twist);
            cmd_vel_pub1.publish(twist1);
            flag = false;
        }
    }



}
