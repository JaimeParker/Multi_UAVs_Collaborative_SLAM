//
// Created by hazyparker on 2022/1/13.
// control UAV to move along waypoints

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Waypoint.h>

mavros_msgs::State current_state; /* NOLINT */
geometry_msgs::PoseStamped current_pose; /* NOLINT */

// callback function for current state
void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}

// callback function for Subscriber for local_pos_sub
void local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

// check if reached a waypoint
bool check_waypoint(const geometry_msgs::PoseStamped &now_pose, const geometry_msgs::PoseStamped &aim_pose){
    // define Point to hold current position and aim position
    geometry_msgs::Point curr, aim;
    curr = now_pose.pose.position;
    aim = aim_pose.pose.position;
    double precision = 0.1;

    // define default return value
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

int main(int argc, char **argv){
    // init ros node
    ros::init(argc, argv, "wayPoint_node");

    // create node handle
    ros::NodeHandle n;

    // define subscriber and publisher
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Subscriber local_pos_sub = n.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_cb);

    // define client and server
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // set set-point publish rate, MUST more than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
	ROS_INFO("wait for fcu...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("fcu connected");

    // set msgs for SetMode
    mavros_msgs::SetMode offboard_mode_set;
    offboard_mode_set.request.custom_mode = "OFFBOARD";

    // set msgs for arming
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // set first waypoint
    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 2;

    // Before entering Offboard mode, you must have already started streaming setpoint
    // Otherwise the mode switch will be rejected
    for(int i = 20; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose1);
        ros::spinOnce();
        rate.sleep();
	ROS_INFO("send point...");
    }

    // record current time
    ros::Time last_request = ros::Time::now();

    // heading for waypoint 1
    while(ros::ok()){
        // launch offboard mode
        if (current_state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0)){
            set_mode_client.call(offboard_mode_set);
            ROS_INFO("Offboard mode enabling...");
            if (offboard_mode_set.response.mode_sent){
                ROS_INFO("Offboard enabled successfully!");
            }
            last_request = ros::Time::now();
        }

        // then arm the UAV
        else{
            if (!current_state.armed && ros::Time::now() - last_request > ros::Duration(5.0)){
                arming_client.call(arm_cmd);
                ROS_INFO("arming the UAV...");
                if (arm_cmd.response.success) ROS_INFO("UAV armed!");
                last_request = ros::Time::now();
            }
        }

//        // publish pose1 information
//        local_pos_pub.publish(pose1);
//        ros::spinOnce();
//
//        // check if reached a waypoint
//        if (check_waypoint(current_pose, pose1)) break;
//        rate.sleep();

        while(ros::ok()){
            // publish pose1 information
            local_pos_pub.publish(pose1);
            ros::spinOnce();

            // check if reached a waypoint
            if (check_waypoint(current_pose, pose1)) break;
            rate.sleep();
        }
    }

//    // hover for a while
//    // hover at set point(pose)
//    for(int i = 100; ros::ok() && i > 0; --i){
//        local_pos_pub.publish(pose1);
//        ros::spinOnce();
//        rate.sleep();
//    }

    // set second waypoint
    geometry_msgs::PoseStamped pose2;
    pose2.pose.position.x = 10;
    pose2.pose.position.y = 10;
    pose2.pose.position.z = 2;

    // heading for waypoint 2
    while(ros::ok()){
        // publish pose1 information
        local_pos_pub.publish(pose2);
        ros::spinOnce();

        // check if reached a waypoint
        if (check_waypoint(current_pose, pose2)) break;
        rate.sleep();
    }

    // set third waypoint
    geometry_msgs::PoseStamped pose3;
    pose3.pose.position.x = 20;
    pose3.pose.position.y = 0;
    pose3.pose.position.z = 2;

    // heading for waypoint 2
    while(ros::ok()){
        // publish pose1 information
        local_pos_pub.publish(pose3);
        ros::spinOnce();

        // check if reached a waypoint
        if (check_waypoint(current_pose, pose3)) break;
        rate.sleep();
    }


    // proceed land
    ROS_INFO("landing");
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";
    while (ros::ok()){
        if(current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0))){
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

