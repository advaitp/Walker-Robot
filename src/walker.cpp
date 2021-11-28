/**
 * @file walker.cpp
 * @brief program to run walker robot autonmously in an environment
 * @author Advait Patole
 *
 * Copyright (c) 2021 Advait Patole
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
#include<walker.hpp>

walker::walker() {
    move.linear.x = 0;
    move.linear.y = 0;
    move.linear.z = 0;
    move.angular.x = 0;
    move.angular.y = 0;
    move.angular.z = 0;

    dis_t = 0.5;
    vel = 0.4;
    rot = 1;
    detected = false;
    right = false;
    left = false;

    velpub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    scansub = nh.subscribe<sensor_msgs::LaserScan>
            ("/scan", 100, &walker::LaserCallback, this);
}

void walker::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (msg->ranges[0] >= dis_t && msg->ranges[29]
                       >= dis_t && msg->ranges[329] >= dis_t) {
        detected = true;
        if (msg->ranges[0] > msg->ranges[179]) {
            right = true;
        } else {
            left = true;
        }
        return;
    }

    detected = false;
}


void walker::navigate() {
    detected = false;

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if (detected) {
            if (left) {
                move.linear.x = 0;
                move.angular.z = rot;
                ROS_INFO_STREAM("Rotating Robot Obstacle detected left");
            } else {
                move.linear.x = 0;
                move.angular.z = -rot;
                ROS_INFO_STREAM("Rotating Robot Obstacle detected right");
            }
        } else {
            move.linear.x = vel;
            move.angular.z = 0;
            ROS_INFO_STREAM("Moving Forward");
        }
        velpub.publish(move);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

walker::~walker() {
    velpub.publish(move);
}
