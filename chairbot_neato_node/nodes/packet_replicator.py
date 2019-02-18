#!/usr/bin/env python
import roslib; roslib.load_manifest("neato_node");
import rospy

from std_msgs.msg import Int8, String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from neato_driver.neato_driver import Botvac

rospy.init_node('packet_tester')

r = rospy.Rate(20) # WHY 20 ???
twist03 = rospy.Publisher('/twist03', Twist, queue_size=30);
motion = None
packet = Twist()

# NOTE: Why is BACKWARD a last but STOP_MOTION a dictionary?

BACKWARD = { 'linear': {'x': 150.0, 'y':0.0, 'z':0.0},
              'angular': {'x': 0.0, 'y':0.0, 'z':0.0}
           }

FORWARD = {}
LEFT = {}
RIGHT = {}

STOP_MOTION = { 
    	  'linear': {'x': 0.0, 'y':0.0, 'z':0.0},
          'angular': {'x': 0.0, 'y':0.0, 'z':0.0}
        }
       

MOTIONS = { 'BACKWARD' : BACKWARD, 
            "FORWARD": FORWARD, 
            'LEFT': LEFT, 
            'RIGHT': RIGHT, 
            'STOP' : STOP_MOTION
           }

# print("We are going to move with ", BACKWARD);

STOP_FLAG = False

# takes the motion message request and sets flags OR motion variables based on it.
def motion_callback(msg):
    global motion, STOP_FLAG
    global MOTIONS
    print "We got a msg", msg.data

    msg = msg.data #just unrwap the command 

    if msg == 'STOP': # we were given the STOP command
        print "We got a STOP"
        STOP_FLAG = True
    else: # we were given a MOTION command
        STOP_FLAG = False
    
    motion = MOTIONS[msg]
    print("The motion is gonna be ", motion)

#requestMotion topic which receives MOTIONs, STOPs and PAUSEs
rospy.Subscriber('/requestMotion03', String, motion_callback, queue_size=10)
rospy.Subscriber('/requestStop03', String, motion_callback, queue_size=10)

while not rospy.is_shutdown():
    if motion is None:
        print "Waiting for motion"
        continue; #try again!

    if STOP_FLAG is True:
        print "Stopping"      
    else:
        print "Moving"

    print "Replicating the packet"
    print motion

    packet.linear.x = motion['linear']['x']
    packet.linear.y = motion['linear']['y']
    packet.linear.z = motion['linear']['z']
    packet.angular.x = motion['angular']['x']
    packet.angular.y = motion['angular']['y']
    packet.angular.z = motion['angular']['z']
    twist03.publish(packet)
    r.sleep()