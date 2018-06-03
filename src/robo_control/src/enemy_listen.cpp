#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
int main(int argc, char** argv){
    ros::init(argc, argv, "enemy_pose_listener");
    ros::NodeHandle nh;
    ros::Publisher pub_enemy_odom =nh.advertise<geometry_msgs::TransformStamped>("enemy/odom_pose", 10);
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(30.0);
    while(ros::ok())
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("odom", "red0", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        pub_enemy_odom.publish(transformStamped);
        ros::spinOnce();
        rate.sleep();
    }
}
