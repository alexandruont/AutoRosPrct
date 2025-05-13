#include "ChasisHandler.h"
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

std::thread ChasisHandler;
RP::vec3 chMovement = RP::vec3(0);

void executeCHTasks();
void initChasisHandler(int argc, char** argv){
    // Initialize the ROS node
    ros::init(argc, argv, "move_forward_node");
    // Create a publisher for the cmd_vel topic
    std::cout << "Ros Inited succesfully!\n";
    ChasisHandler = std::thread(executeCHTasks);
}

void closeChasisHandler(){
    while(!ChasisHandler.joinable())
        sleep(1);
    ChasisHandler.join();
}

void getMapData(std::vector<uint8_t>& buffer){
    // Simulate getting map data
    buffer.resize(256 * 256 * 3);
    std::fill(buffer.begin(), buffer.end(), 255);
}

void executeCHTasks(){
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    ros::Rate rate(10);
    geometry_msgs::Twist move_cmd;
    while(!closeProgram.load()){
        move_cmd.linear.x = chMovement.x;
        move_cmd.linear.y = chMovement.y;
        move_cmd.angular.z = chMovement.z;
        cmd_vel_pub.publish(move_cmd);
        ros::spinOnce();
        rate.sleep();
    }
    move_cmd.linear.x = 0;
    move_cmd.linear.y = 0;
    move_cmd.angular.z = 0;
    cmd_vel_pub.publish(move_cmd);
    ros::spinOnce();
}