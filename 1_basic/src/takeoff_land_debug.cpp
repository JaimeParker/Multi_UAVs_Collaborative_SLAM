//
// Created by hazyparker on 2022/1/13.
// realize mode switching and anto takeoff and landing
// try to arm the plane firstly, then change it into off-board mode

#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Waypoint.h>

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

int main(int argc, char **argv){
    // init ros node
    ros::init(argc, argv, "offb_node");

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

    //the set-point publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("wait for fcu connecting...");
    }
    ROS_INFO("fcu connected successfully");

    // set pose
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few set-points before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ROS_INFO("Off boarding");
    while(ros::ok()){
        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if (current_state.mode != "OFFBOARD") ROS_INFO("Off board mode wasn't launched");
            ROS_INFO("trying to arm UAV");
            if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Off board enabled");
            }
            last_request = ros::Time::now();
        }

        local_pos_pub.publish(pose);
        // wait until reach set point
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
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // proceed landing process
    ROS_INFO("landing");
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";
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

    return 0;
}

