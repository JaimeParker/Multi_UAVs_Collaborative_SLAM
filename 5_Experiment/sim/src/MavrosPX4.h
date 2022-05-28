//
// Created by hazyparker on 22-5-27.
//

#ifndef BASIC_MAVROSPX4_H
#define BASIC_MAVROSPX4_H

// include headers
#include <cstring>
#include <string>

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

class MavrosPX4 {
public:
    // pointer
    MavrosPX4 *data_pointer = this;

    // ros node name
    string NodeName;

    // topic names
    string MavrosState;          // "mavros/state"
    string MavrosLocalPose;      // "mavros/local_position/pose"
    string MavrosSetPointLocal;  // "mavros/setpoint_position/local"
    string MavrosCmdArm;         // "mavros/cmd/arming"
    string MavrosSetMode;        // "mavros/set_mode"

    // Mavros msgs: current state
    mavros_msgs::State current_state;

    // geometry msgs: current pose
    geometry_msgs::PoseStamped current_pose;

    // frequency for sending commands
    double frequency;


    MavrosPX4();

    /**
     * callback function for subscribing state message
     * @param msg
     */
    static void state_cb(const mavros_msgs::State::ConstPtr& msg, void *userdata);

    static void local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg, void *userdata);

    void InitROSNode(int argc, char **argv);



    static void Offboard();
};


#endif //BASIC_MAVROSPX4_H
