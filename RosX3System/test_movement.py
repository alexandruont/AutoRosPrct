#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_robot():
    # Initialize the ROS node
    rospy.init_node('cmd_vel_publisher', anonymous=True)

    # Create a publisher for the /cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create a Twist message
    move_cmd = Twist()
    move_cmd.linear.x = 1  # Move forward at 0.2 m/s
    move_cmd.angular.z = 0.0 # No rotation

    # Publish the velocity command
    rospy.loginfo("Moving the robot forward...")
    rate = rospy.Rate(10)  # 10 Hz

    # Move for 5 seconds
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < 5:
        cmd_vel_pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot
    move_cmd.linear.x = 0.0
    rospy.loginfo("Stopping the robot.")
    cmd_vel_pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
