#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h" //??
#include "nlink_parser/LinktrackAnchorframe0.h"
#include "std_msgs/Int32.h"
#include "nlink_parser/LinktrackTagframe0.h"
#include "nlink_parser/LinktrackNodeframe2.h"
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
//using namespace Eigen;

float pos_uwb[3]={0,0,0};

ros::Publisher vision_pub;

void uwb_to_fcu();

void cb_uwb(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg)
{
	nlink_parser::LinktrackNodeframe2 uwb_data;
	uwb_data = *msg;
	pos_uwb[0] =  uwb_data.pos_3d[0];
	pos_uwb[1] =  uwb_data.pos_3d[1];
	pos_uwb[2] =  uwb_data.pos_3d[2];
}


void uwb_to_fcu()
{
	geometry_msgs::PoseStamped vision;

        vision.pose.position.x = pos_uwb[0];
        vision.pose.position.y = pos_uwb[1];
        vision.pose.position.z = pos_uwb[2];

        //vision.pose.orientation.x = q_uwb.x();
        //vision.pose.orientation.y = q_uwb.y();
        //vision.pose.orientation.z = q_uwb.z();
        //vision.pose.orientation.w = q_uwb.w();

	vision.header.stamp = ros::Time::now();
	vision_pub.publish(vision);
	
}



int main(int argc,char **argv)
{
	ros::init(argc,argv,"uwb0");
	ros::NodeHandle n("");
	ros::NodeHandle nh("~");

	ros::Subscriber uwb_sub = n.subscribe("nlink_linktrack_nodeframe2",10,cb_uwb);
	vision_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 100);

	ros::Rate loop_rate(50);

	while(ros::ok())
	{
		ros::spinOnce();
		uwb_to_fcu();
		loop_rate.sleep();
	}  //无法在回调函数里发布话题，报错函数里没有定义vel_pub!只能在main里面发布了

	return 0;
}
