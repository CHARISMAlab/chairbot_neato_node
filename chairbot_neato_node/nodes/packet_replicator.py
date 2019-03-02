#!/usr/bin/env python
import roslib; roslib.load_manifest("neato_node");
import rospy

from what_is_my_name import chairbot_number
from std_msgs.msg import Int8, String
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from neato_driver.neato_driver import Botvac

rospy.init_node('packet_tester')


#TODO: make this whole file in a class just like twist_listener.py

chairbot_number = chairbot_number()
chairMovement_topic_name = 'chairMovement' + chairbot_number
requestMotion_topic_name = 'requestMotion' + chairbot_number
stopMotion_topic_name = 'requestStop' + chairbot_number

#publish to the topic for sending Twist messages which the neato node 
#will use for actually moving
chairMovementTopic = rospy.Publisher(chairMovement_topic_name, Twist, queue_size=30);

r = rospy.Rate(20) # WHY 20 ???

motion = None
packet = Twist()

BACKWARD = { 
              'linear': {'x': 150.0, 'y':0.0, 'z':0.0},
              'angular': {'x': 0.0, 'y':0.0, 'z':0.0}
           }

FORWARD = { 
            'linear': {'x': -150.0, 'y':0.0, 'z':0.0},
            'angular': {'x': 0.0, 'y':0.0, 'z':0.0}
         }
LEFT = { 
            'linear': {'x': 0.0, 'y':0.0, 'z':0.0},
            'angular': {'x': 0.0, 'y':0.0, 'z':50.0}
}
RIGHT = {
            'linear': {'x': 0.0, 'y':0.0, 'z':0.0},
            'angular': {'x': 0.0, 'y':0.0, 'z':-50.0}
}

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
rospy.Subscriber(requestMotion_topic_name, String, motion_callback, queue_size=10)
rospy.Subscriber(stopMotion_topic_name, String, motion_callback, queue_size=10)

while not rospy.is_shutdown():
    if motion is None:
        #print "Waiting for motion"
        continue; #try again!

    if STOP_FLAG is True:
        #print "Stopping"
        pass;
    else:
        #print "Moving"
        pass;

    print "Replicating the packet"
    print motion

    packet.linear.x = motion['linear']['x']
    packet.linear.y = motion['linear']['y']
    packet.linear.z = motion['linear']['z']
    packet.angular.x = motion['angular']['x']
    packet.angular.y = motion['angular']['y']
    packet.angular.z = motion['angular']['z']
    chairMovementTopic.publish(packet)
    r.sleep()
