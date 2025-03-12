#include "ChasisHandler.h"
#include <iostream>
#include "CommonData.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

std::thread ChasisHandler;

void executeCHTasks();
void initChasisHandler(int argc, char** argv){
    // Initialize the ROS node
    ros::init(argc, argv, "move_forward_node");
    // Create a publisher for the cmd_vel topic

    ChasisHandler = std::thread(executeCHTasks);
}

void closeChasisHandler(){
    ChasisHandler.join();
}

void executeCHTasks(){
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    ros::Rate rate(10);
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = 0.5;  // Move forward at 0.5 m/s
    move_cmd.angular.z = 0.0; // No rotation
    while(!closeProgram.load()){
        cmd_vel_pub.publish(move_cmd);
        
        ros::spinOnce();
        rate.sleep();
    }
}