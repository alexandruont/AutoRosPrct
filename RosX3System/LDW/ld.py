#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def callback(scan_data):
    try:
       print("Distante: ", scan_data.ranges)
    except Exception as e:
        rospy.logerr(f"Eroare la citirea datelor Lidar: {e}")

def listener():
    rospy.init_node('lidar_listener', anonymous=True)
    # abonare la topicul '/scan'; asigura te ca acesta este cel corect pentru senzorul tau
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()  # mentine nodul activ

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    