#include <ros/ros.h>
#include <nav_msgs/Odometry.h>


class NN{
public:
    ros::NodeHandle *pnh;
    NN(ros::NodeHandle* _pnh){
            pnh = _pnh;
    };
    void cb_odom(const nav_msgs::Odometry &msg);
};

void NN::cb_odom(const nav_msgs::Odometry &msg){
    ROS_INFO("SUB  %lf  %f  %f",msg.header.stamp,msg.pose.pose.position.x,msg.pose.pose.position.y);
}

void cc(const nav_msgs::Odometry &msg){
    ROS_INFO("SUB %lf  %f  %f",msg.header.stamp.toSec(),msg.pose.pose.position.x,msg.pose.pose.position.y);
}

int main(int argc,char** argv){
    ros::init(argc,argv,"nav_test");
    ros::NodeHandle nh;

    // NN n(&nh);
    ros::Subscriber cb_tar_pose = nh.subscribe("odom", 1,&cc);

    ros::spin();
}