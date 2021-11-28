#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include<walker.hpp>

walker::walker(){
    move.linear.x = 0;
    move.linear.y = 0;
    move.linear.z = 0;
    move.angular.x = 0;
    move.angular.y = 0;
    move.angular.z = 0;

    dis_thresh = 0.4;
    vel = 0.4;
    rot = 1;
    detected = false;

    velpub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    scansub = nh.subscribe<sensor_msgs::LaserScan>("/scan",100, &walker::LaserCallback,this);
}

void walker::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    for (auto dist : msg->ranges) {
      if (dist < dis_thresh) {
        detected = true;
        ROS_INFO_STREAM("Obstacle detected in laser range");
        return;
        }
      }

      detected = false;
}


void walker::navigate(){
    detected = false;

    ros::Rate loop_rate(10);
    while (ros::ok()){
        if(detected){
            move.linear.x = 0;
            move.angular.z = rot;
            ROS_INFO_STREAM("Rotating Robot Obstacle detected");
        }

        else{
            move.linear.x = vel;
            move.angular.z = 0;
            ROS_INFO_STREAM("Moving Forward");
        }

        velpub.publish(move);
        ros::spinOnce();

        loop_rate.sleep();

    }

}

walker::~walker(){
    velpub.publish(move);
}
