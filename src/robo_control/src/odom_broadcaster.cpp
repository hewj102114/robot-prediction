#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
using namespace std;





class OdomBroadcaster{
public:
	OdomBroadcaster(){
		pnh=new ros::NodeHandle("");
		pub_odom=pnh->advertise<nav_msgs::Odometry>("odom",1);
	};

	ros::NodeHandle* pnh;
	ros::Publisher pub_odom;
	ros::Subscriber sub_ukf_pose;
	tf2_ros::TransformBroadcaster odom_tf2;
    
	
	void cb_ukf_pose(const nav_msgs::OdometryConstPtr& msg);
};

void OdomBroadcaster::cb_ukf_pose(const nav_msgs::OdometryConstPtr& msg){
	double cur_yaw=tf::getYaw(msg->pose.pose.orientation);

	nav_msgs::Odometry odom_msg;
	odom_msg.header=msg->header;
	odom_msg.header.frame_id="odom";
	odom_msg.child_frame_id="base_link";
	odom_msg.pose.pose.position.x=msg->pose.pose.position.x+cos(cur_yaw)*0.13;
	odom_msg.pose.pose.position.y=msg->pose.pose.position.y+sin(cur_yaw)*0.13;
	odom_msg.pose.pose.position.z=0;
	odom_msg.pose.pose.orientation=msg->pose.pose.orientation;
	odom_msg.twist=msg->twist;
	pub_odom.publish(odom_msg);
	
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = msg->header.stamp;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.x=msg->pose.pose.position.x+cos(cur_yaw)*0.13;
	odom_trans.transform.translation.y=msg->pose.pose.position.y+sin(cur_yaw)*0.13;
	odom_trans.transform.translation.z=0;
	odom_trans.transform.rotation=msg->pose.pose.orientation;
	odom_tf2.sendTransform(odom_trans);

}






int main(int argc,char** argv)
{
  ros::init(argc,argv,"odom_broadcaster");
  ros::NodeHandle nh;
  
  OdomBroadcaster odom_broadcaster;
  ros::Subscriber sub_message = nh.subscribe("ukf/pos", 1, &OdomBroadcaster::cb_ukf_pose, &odom_broadcaster);
  ros::spin();
}