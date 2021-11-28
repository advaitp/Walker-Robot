#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    ROS_INFO("LaserScan (val,angle)=(%f", msg->ranges[0]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "topic_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber scanSub;
    
    scanSub = nh.subscribe<sensor_msgs::LaserScan>(&walker::LaserCallback,this);
    ros::spin();
    return 0;
}
