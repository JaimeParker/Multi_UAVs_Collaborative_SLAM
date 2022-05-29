//
// Created by hazy parker on 22-5-27.
// use KeyBoard to control UAV in PX4 SITL

// include headers
#include <iostream>
#include <cmath>
#include <termios.h>
#include <cstdio>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Waypoint.h>


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

using namespace std;

// record current state
mavros_msgs::State current_state; /* NOLINT */
// local variable will be warned, adding a /* NOLINT */ to avoid this

// callback function for Subscriber stats_sub
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// record current pose
geometry_msgs::PoseStamped current_pose; /* NOLINT */

// callback function for Subscriber for local_pos_sub
void local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
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

void Land(ros::ServiceClient &set_mode_client, mavros_msgs::SetMode &land_set_mode,
          ros::Rate &rate){
    // proceed landing process
    ros::Time last_request = ros::Time::now();
    ROS_INFO("landing");

    while (ros::ok()){
        if( current_state.mode != "AUTO.LAND" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(land_set_mode) &&
                land_set_mode.response.mode_sent){
                ROS_INFO("Land enabled");
            }
            last_request = ros::Time::now();
        }
        if(!current_state.armed){
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void velocityTuning(mavros_msgs::PositionTarget &positionTarget, int cmd){
    /**
     * CMD value means:
     * 1, to add velocity in x
     * 2, to dec velocity in x
     * 3, to add velocity in y
     * 4, to dec velocity in y
     * 5, to add velocity in z
     * 6, to dec velocity in z
     * 0, to set all velocity as 0, hold still
     * 7, to add yaw
     * 8, to dec yaw
     */

    // variables to hold value
    double vx, vy, vz;
    double yaw;

    vx = positionTarget.velocity.x;
    vy = positionTarget.velocity.y;
    vz = positionTarget.velocity.z;
    yaw = positionTarget.yaw;

    if (cmd == 1){
        vx = vx + 0.1;
        if (vx > 1) vx = 1;
    }

    if (cmd == 2){
        vx = vx - 0.1;
        if (vx < -1) vx = -1;
    }

    if (cmd == 3){
        vy = vy + 0.1;
        if (vy > 1) vy = 1;
    }

    if (cmd == 4){
        vy = vy - 0.1;
        if (vy < -1) vy = -1;
    }

    if (cmd == 5){
        vz = vz + 00.1;
        if (vz > 5) vz = 5;
    }

    if (cmd == 6){
        vz = vz - 0.1;
        if (vz < -2) vz = -2;
    }

    if (cmd == 7){
        yaw = yaw + 0.1;
        if (yaw > 1) yaw = 1;
    }

    if (cmd == 8){
        yaw = yaw - 0.1;
        if (yaw < -1) yaw = -1;
    }

    if (cmd == 0){
        vx = 0;
        vy = 0;
        vz = 0;
        yaw = 0;
    }

    if (cmd == -1){
        positionTarget.velocity.x = vx;
        positionTarget.velocity.y = vy;
        positionTarget.velocity.z = vz;
        positionTarget.yaw = yaw;
    }

    positionTarget.velocity.x = vx;
    positionTarget.velocity.y = vy;
    positionTarget.velocity.z = vz;
    positionTarget.yaw = yaw;
}

int main(int argc, char **argv){
    // init ros node
    ros::init(argc, argv, "KeyBoardControl");

    // create node handle
    ros::NodeHandle nh;

    // define subscribers and clients
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose",10,local_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);

    //the set-point publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("wait for fcu connecting...");
    }
    ROS_INFO("fcu connected successfully");
    puts("Link up with FCU! Try to connect with KeyBoard...");

    // set pose
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 3;

    //send a few set-points before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // define Offboard config
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // define Arm config
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // define Land config
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    // mavros set point raw local
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.velocity.x = 0;
    positionTarget.velocity.y = 0;
    positionTarget.velocity.z = 0;
    positionTarget.yaw = 0;

    char c = 'e';

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
    puts("Press T to shift to offboard mode and take off");
    puts("Press L to proceed Landing");
    puts("Press H to hold position");

    bool flag = false;

    for (int i = 0; i < 10; i++){
        local_pos_pub.publish(current_pose);
        ros::spinOnce();
        rate.sleep();
    }

    for(;;){
        // get current pose
//        geometry_msgs::PoseStamped curr;
//        curr.pose.position.x = current_pose.pose.position.x;
//        curr.pose.position.y = current_pose.pose.position.y;
//        curr.pose.position.z = current_pose.pose.position.z;

        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0){
            perror("read():");
            exit(-1);
        }

        switch(c){
//            default:
//                ROS_INFO("using default position");
//                velocityTuning(positionTarget, -1);
//                flag = true;
//                break;
            case KEYCODE_T:
                ROS_INFO("Command Offboard mode RECEIVED!");
                Offboard(set_mode_client, offb_set_mode, arm_cmd,
                         arming_client, local_pos_pub, pose, rate);
                velocityTuning(positionTarget, 0);
                flag = true;
                break;
            case KEYCODE_L:
                ROS_INFO("Command Land mode RECEIVED!");
                Land(set_mode_client, land_set_mode, rate);
                ROS_INFO("Proceed Landing Operation!");
                break;
            case KEYCODE_LEFT:
                ROS_INFO("Command Turning RECEIVED!");
                velocityTuning(positionTarget, 7);
                flag = true;
                break;
            case KEYCODE_RIGHT:
                ROS_INFO("Command Turning RECEIVED!");
                velocityTuning(positionTarget, 8);
                flag = true;
                break;
            case KEYCODE_UP:
                ROS_INFO("Command Add thrust RECEIVED!");
                velocityTuning(positionTarget, 5);
                flag = true;
                break;
            case KEYCODE_DOWN:
                ROS_INFO("Command Dec thrust RECEIVED!");
                velocityTuning(positionTarget, 6);
                flag = true;
                break;
            case KEYCODE_W:
                ROS_INFO("Command Add forward speed RECEIVED!");
                velocityTuning(positionTarget, 1);
                flag = true;
                break;
            case KEYCODE_S:
                ROS_INFO("Command Dec forward speed RECEIVED!");
                velocityTuning(positionTarget, 2);
                flag = true;
                break;
            case KEYCODE_A:
                ROS_INFO("Command Add flank speed RECEIVED!");
                velocityTuning(positionTarget, 3);
                flag = true;
                break;
            case KEYCODE_D:
                ROS_INFO("Command Dec flank speed RECEIVED!");
                velocityTuning(positionTarget, 4);
                flag = true;
                break;
            case KEYCODE_H:
                ROS_INFO("Command Hold RECEIVED!");
                velocityTuning(positionTarget, 0);
                flag = true;
                break;

            case KEYCODE_Q:
                ROS_DEBUG("quit");
                return 0;
        }

        if(flag){
	        flag = false;
            for (int i = 0; i < 100; i++) {
                raw_local_pub.publish(positionTarget);

            }

        }
    }

}



