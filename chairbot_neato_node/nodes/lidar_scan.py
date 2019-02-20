#!/usr/bin/env python

import roslib; roslib.load_manifest("chairbot_neato_node")
import rospy, time

from math import sin,cos,radians
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from chairbot_neato_driver.chairbot_neato_driver import Botvac

import socket
import re
global neato_number
hostname = socket.gethostname()
robot_id = re.search(r"\d+(\.\d+)?", hostname)
neato_number = 'pi'+robot_id.group(0)
print(neato_number)

RATE = 40

def publish_scans(robot):
    raw_lidar_data = robot.getScanRanges()
    if len(raw_lidar_data) == 360:

        lidar_scan = LaserScan()

        angle_min = radians(raw_lidar_data[0][0])
        angle_max = radians(raw_lidar_data[-1][0])
        angle_increment = (angle_max - angle_min)/(len(raw_lidar_data)-1)
        scan_time = 1.0/RATE

        range_min = 100
        range_max = 0
        ranges = []
        intensities = []

        for i in range(len(raw_lidar_data)):
            if raw_lidar_data[i][1] > range_max:
                range_max = raw_lidar_data[i][1]
            if raw_lidar_data[i][1] < range_min:
                range_min = raw_lidar_data[i][1]
            ranges.append(raw_lidar_data[i][1])
            intensities.append(raw_lidar_data[i][2])

        lidar_scan.header.frame_id = 'map'
        lidar_scan.angle_min = angle_min
        lidar_scan.angle_max = angle_max
        lidar_scan.angle_increment = angle_increment
        lidar_scan.scan_time = scan_time
        lidar_scan.range_min = range_min
        lidar_scan.range_max = range_max
        lidar_scan.ranges = ranges
        lidar_scan.intensities = intensities

        pub_scan.publish(lidar_scan)

if __name__ == "__main__":
    rospy.init_node('lidar_scan'+neato_number, anonymous=True)
    pub_scan = rospy.Publisher("/scan/"+neato_number, LaserScan, queue_size=10)
    robot_port = rospy.get_param('~neato_port', "/dev/neato_port")
    rospy.loginfo("Using port: %s"%(robot_port))
    robot = Botvac(robot_port)
    r = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        publish_scans(robot)
        r.sleep()
    # shut down
    robot.shutdown()
