//
// Created by hazyparker on 22-5-27.
//

#include "MavrosPX4.h"
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

MavrosPX4::MavrosPX4() {
    // ros node name
    NodeName = "MavrosPX4Connect";

    // set default topic name
    MavrosState = "mavros/state";
    MavrosLocalPose = "mavros/local_position/pose";
    MavrosSetPointLocal = "mavros/setpoint_position/local";
    MavrosCmdArm = "mavros/cmd/arming";
    MavrosSetMode = "mavros/set_mode";

    // set frequency for sending messages
    frequency = 20.0;
}

void MavrosPX4::state_cb(const mavros_msgs::State::ConstPtr &msg, void *userdata) {
    auto *data = (MavrosPX4 *) userdata;

    data->current_state = *msg;
}

void MavrosPX4::local_cb(const geometry_msgs::PoseStamped::ConstPtr &msg,
                         void *userdata) {
    auto *data = (MavrosPX4 *) userdata;

    data->current_pose = *msg;
}

void local_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){

}

void MavrosPX4::InitROSNode(int argc, char **argv) {
    // init ros node
    ros::init(argc, argv, NodeName);

    // create node handle
    ros::NodeHandle nh;

    // define subscribers and publishers
//    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
//            (MavrosState, 10, state_cb(nullptr, data_pointer));
//    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
//            (MavrosLocalPose, 10, local_cb);
    // FIXME: callback function needs one parameter
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            (MavrosSetPointLocal, 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (MavrosCmdArm);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (MavrosSetMode);

}
