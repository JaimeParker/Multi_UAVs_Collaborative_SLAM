//
// Created by hazyparker on 2022/1/20.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Waypoint.h>

// each uav should have its own current_state for state checking
mavros_msgs::State current_state; /* NOLINT */

// each uav should have its own current_pose for pose following
geometry_msgs::PoseStamped current_pose; /* NOLINT */
geometry_msgs::PoseStamped current_pose1; /* NOLINT */
geometry_msgs::PoseStamped current_pose2; /* NOLINT */
geometry_msgs::PoseStamped current_pose3; /* NOLINT */
geometry_msgs::PoseStamped current_pose4; /* NOLINT */
geometry_msgs::PoseStamped current_pose5; /* NOLINT */

// each uav should have its own state callback function for state tracking
// callback function for current state
void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}

// each uav should have its own local pos callback function for pos tracking
// callback function for Subscriber for local_pos_sub
void local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}
void local_cb1(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose1 = *msg;
}
void local_cb2(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose2 = *msg;
}
void local_cb3(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose3 = *msg;
}
void local_cb4(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose4 = *msg;
}
void local_cb5(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose5 = *msg;
}

// check if reached a waypoint
bool check_waypoint(const geometry_msgs::PoseStamped &now_pose, const geometry_msgs::PoseStamped &aim_pose){
    // define Point to hold current position and aim position
    geometry_msgs::Point curr, aim;
    curr = now_pose.pose.position;
    aim = aim_pose.pose.position;
    double precision = 0.2;

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

    // UAV 1
    // create node handle
    ros::NodeHandle n1;
    // define subscriber and publisher
    ros::Subscriber state_sub1 = n1.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub1 = n1.advertise<geometry_msgs::PoseStamped>
            ("uav1/mavros/setpoint_position/local", 10);
    ros::Subscriber local_pos_sub1 = n1.subscribe<geometry_msgs::PoseStamped>
            ("uav1/mavros/local_position/pose", 10, local_cb1);
    // define client and server
    ros::ServiceClient arming_client1 = n1.serviceClient<mavros_msgs::CommandBool>
            ("uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client1 = n1.serviceClient<mavros_msgs::SetMode>
            ("uav1/mavros/set_mode");

    // UAV 2
    // create node handle
    ros::NodeHandle n2;
    // define subscriber and publisher
    ros::Subscriber state_sub2 = n2.subscribe<mavros_msgs::State>
            ("uav2/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub2 = n2.advertise<geometry_msgs::PoseStamped>
            ("uav2/mavros/setpoint_position/local", 10);
    ros::Subscriber local_pos_sub2 = n2.subscribe<geometry_msgs::PoseStamped>
            ("uav2/mavros/local_position/pose", 10, local_cb2);
    // define client and server
    ros::ServiceClient arming_client2 = n2.serviceClient<mavros_msgs::CommandBool>
            ("uav2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client2 = n2.serviceClient<mavros_msgs::SetMode>
            ("uav2/mavros/set_mode");

    // UAV 3
    // create node handle
    ros::NodeHandle n3;
    // define subscriber and publisher
    ros::Subscriber state_sub3 = n3.subscribe<mavros_msgs::State>
            ("uav3/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub3 = n3.advertise<geometry_msgs::PoseStamped>
            ("uav3/mavros/setpoint_position/local", 10);
    ros::Subscriber local_pos_sub3 = n3.subscribe<geometry_msgs::PoseStamped>
            ("uav3/mavros/local_position/pose", 10, local_cb3);
    // define client and server
    ros::ServiceClient arming_client3 = n3.serviceClient<mavros_msgs::CommandBool>
            ("uav3/mavros/cmd/arming");
    ros::ServiceClient set_mode_client3 = n3.serviceClient<mavros_msgs::SetMode>
            ("uav3/mavros/set_mode");

    // UAV 4
    // create node handle
    ros::NodeHandle n4;
    // define subscriber and publisher
    ros::Subscriber state_sub4 = n4.subscribe<mavros_msgs::State>
            ("uav4/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub4 = n4.advertise<geometry_msgs::PoseStamped>
            ("uav4/mavros/setpoint_position/local", 10);
    ros::Subscriber local_pos_sub4 = n4.subscribe<geometry_msgs::PoseStamped>
            ("uav4/mavros/local_position/pose", 10, local_cb4);
    // define client and server
    ros::ServiceClient arming_client4 = n4.serviceClient<mavros_msgs::CommandBool>
            ("uav4/mavros/cmd/arming");
    ros::ServiceClient set_mode_client4 = n4.serviceClient<mavros_msgs::SetMode>
            ("uav4/mavros/set_mode");

    // UAV 5
    // create node handle
    ros::NodeHandle n5;
    // define subscriber and publisher
    ros::Subscriber state_sub5 = n5.subscribe<mavros_msgs::State>
            ("uav5/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub5 = n5.advertise<geometry_msgs::PoseStamped>
            ("uav5/mavros/setpoint_position/local", 10);
    ros::Subscriber local_pos_sub5 = n5.subscribe<geometry_msgs::PoseStamped>
            ("uav5/mavros/local_position/pose", 10, local_cb5);
    // define client and server
    ros::ServiceClient arming_client5 = n5.serviceClient<mavros_msgs::CommandBool>
            ("uav5/mavros/cmd/arming");
    ros::ServiceClient set_mode_client5 = n5.serviceClient<mavros_msgs::SetMode>
            ("uav5/mavros/set_mode");

    // set set-point publish rate, MUST more than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to fcu...");
    }
    ROS_INFO("fcu connected successfully!");

    // set msgs for SetMode
    mavros_msgs::SetMode offboard_mode_set;
    offboard_mode_set.request.custom_mode = "OFFBOARD";

    // set msgs for arming
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // set first waypoint
    geometry_msgs::PoseStamped pose_uav0;
    pose_uav0.pose.position.x = 0;
    pose_uav0.pose.position.y = 0;
    pose_uav0.pose.position.z = 6;
    geometry_msgs::PoseStamped pose_uav1;
    pose_uav1.pose.position.x = 1;
    pose_uav1.pose.position.y = 0;
    pose_uav1.pose.position.z = 6;
    geometry_msgs::PoseStamped pose_uav2;
    pose_uav2.pose.position.x = -1;
    pose_uav2.pose.position.y = 0;
    pose_uav2.pose.position.z = 6;
    geometry_msgs::PoseStamped pose_uav3;
    pose_uav3.pose.position.x = 0;
    pose_uav3.pose.position.y = 0;
    pose_uav3.pose.position.z = 5;
    geometry_msgs::PoseStamped pose_uav4;
    pose_uav4.pose.position.x = 0;
    pose_uav4.pose.position.y = 0;
    pose_uav4.pose.position.z = 4;
    geometry_msgs::PoseStamped pose_uav5;
    pose_uav5.pose.position.x = 0;
    pose_uav5.pose.position.y = 0;
    pose_uav5.pose.position.z = 3;

    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(pose_uav0);
        local_pos_pub1.publish(pose_uav1);
        local_pos_pub2.publish(pose_uav2);
        local_pos_pub3.publish(pose_uav3);
        local_pos_pub4.publish(pose_uav4);
        local_pos_pub5.publish(pose_uav5);

        ros::spinOnce();
        rate.sleep();
    }

    // proceed taking off procedure
    // record current time
    ros::Time last_request = ros::Time::now();

    // heading for waypoint 1
    while(ros::ok()){
        // launch offboard mode
        if (current_state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0)){
            set_mode_client0.call(offboard_mode_set);
            set_mode_client1.call(offboard_mode_set);
            set_mode_client2.call(offboard_mode_set);
            set_mode_client3.call(offboard_mode_set);
            set_mode_client4.call(offboard_mode_set);
            set_mode_client5.call(offboard_mode_set);

            ROS_INFO("Offboard mode enabling...");
            if (offboard_mode_set.response.mode_sent){
                ROS_INFO("Offboard enabled successfully!");
            }
            last_request = ros::Time::now();
        }

            // then arm the UAV
        else{
            if (!current_state.armed && ros::Time::now() - last_request > ros::Duration(5.0)){
                arming_client0.call(arm_cmd);
                arming_client1.call(arm_cmd);
                arming_client2.call(arm_cmd);
                arming_client3.call(arm_cmd);
                arming_client4.call(arm_cmd);
                arming_client5.call(arm_cmd);

                ROS_INFO("arming the UAV...");
                if (arm_cmd.response.success) ROS_INFO("UAV armed!");
                last_request = ros::Time::now();
            }
        }

        local_pos_pub0.publish(pose_uav0);
        // publish pose1 information
        /**
         * Be advised for coordinate transformation!
         * For example, home position of uav1 is (1, 0, 0) in x, y, z order;
         * So the relative coordinate for uav1 will decrease one in x axis;
         */
        pose_uav1.pose.position.x = current_pose.pose.position.x + 1 - 1;
        pose_uav1.pose.position.y = current_pose.pose.position.y;
        pose_uav1.pose.position.z = current_pose.pose.position.z;
        pose_uav2.pose.position.x = current_pose.pose.position.x - 1;
        pose_uav2.pose.position.y = current_pose.pose.position.y - 1;
        pose_uav2.pose.position.z = current_pose.pose.position.z;
        pose_uav3.pose.position.x = current_pose.pose.position.x + 1;
        pose_uav3.pose.position.y = current_pose.pose.position.y;
        pose_uav3.pose.position.z = current_pose.pose.position.z - 1;
        pose_uav4.pose.position.x = current_pose.pose.position.x;
        pose_uav4.pose.position.y = current_pose.pose.position.y + 1;
        pose_uav4.pose.position.z = current_pose.pose.position.z - 2;
        pose_uav5.pose.position.x = current_pose.pose.position.x;
        pose_uav5.pose.position.y = current_pose.pose.position.y - 2;
        pose_uav5.pose.position.z = current_pose.pose.position.z - 3;
        // FIXME: there should be a better method for coordinate set

        local_pos_pub1.publish(pose_uav1);
        local_pos_pub2.publish(pose_uav2);
        local_pos_pub3.publish(pose_uav3);
        local_pos_pub4.publish(pose_uav4);
        local_pos_pub5.publish(pose_uav5);

        ros::spinOnce();

        // check if reached a waypoint
        if (check_waypoint(current_pose, pose_uav0)) break;
        rate.sleep();
    }

    // hover for a while
    // hover at set point(pose)
    ROS_INFO("ready to hover...");
    for(int i = 50; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(pose_uav0);
        local_pos_pub1.publish(pose_uav1);
        local_pos_pub2.publish(pose_uav2);
        local_pos_pub3.publish(pose_uav3);
        local_pos_pub4.publish(pose_uav4);
        local_pos_pub5.publish(pose_uav5);

        ros::spinOnce();
        rate.sleep();
    }

    // change formation
    while(ros::ok()){
        // publish pose1 information
        local_pos_pub0.publish(pose_uav0);
        pose_uav1.pose.position.x = current_pose.pose.position.x-1;
        pose_uav1.pose.position.y = current_pose.pose.position.y;
        pose_uav1.pose.position.z = current_pose.pose.position.z - 1;

        pose_uav2.pose.position.x = current_pose.pose.position.x;
        pose_uav2.pose.position.y = current_pose.pose.position.y-1;
        pose_uav2.pose.position.z = current_pose.pose.position.z - 1;

        pose_uav3.pose.position.x = current_pose.pose.position.x+1;
        pose_uav3.pose.position.y = current_pose.pose.position.y;
        pose_uav3.pose.position.z = current_pose.pose.position.z - 1;

        pose_uav4.pose.position.x = current_pose.pose.position.x;
        pose_uav4.pose.position.y = current_pose.pose.position.y+1 + 1;
        pose_uav4.pose.position.z = current_pose.pose.position.z + 1;

        pose_uav5.pose.position.x = current_pose.pose.position.x;
        pose_uav5.pose.position.y = current_pose.pose.position.y-2 - 1;
        pose_uav5.pose.position.z = current_pose.pose.position.z + 2;

        ros::spinOnce();

        local_pos_pub1.publish(pose_uav1);
        local_pos_pub2.publish(pose_uav2);
        local_pos_pub3.publish(pose_uav3);
        local_pos_pub4.publish(pose_uav4);
        local_pos_pub5.publish(pose_uav5);

        rate.sleep();
    }

    return 0;
}

