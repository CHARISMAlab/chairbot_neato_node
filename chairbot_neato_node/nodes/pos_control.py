#!/usr/bin/env python

import roslib; roslib.load_manifest("chairbot_neato_node")
import rospy, time
from subprocess import check_output

from math import sin,cos,atan2
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8
from chairbot_neato_driver.chairbot_neato_driver import Botvac

import socket
import re
global chairbot_number
hostname = socket.gethostname()
chair_id = re.search(r"\d+(\.\d+)?", hostname)
chairbot_number = chair_id.group(0)

class NeatoNode:

    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('teleop'+chairbot_number, anonymous=True)
        self._port = rospy.get_param('~neato_port', "/dev/neato_port")
        rospy.loginfo("Using port: %s"%(self._port))
        self._robot = Botvac(self._port)
        rospy.Subscriber("/joy"+chairbot_number, Joy, self.joy_handler, queue_size=10)

        self._joystick_axes = (-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0)
        self._joystick_buttons = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self._speed = 0
        self._distance = 20
        self._speed_set = 0
        self._speed_ramp = 0
        self._last_x = 0
        self._last_y = 0
        self._x_ramp = 0
        self._y_ramp = 0

        # FIDUCIAL MARKER variables
        self.fiducial_marker_file_path = '/home/charisma/processing-3.5.3'
        self.fiducial_marker_file_name = 'output1.txt'
        self.fiducial_marker_data = {'time':None,'x':None,'y':None,'angle':None}
        self.fiducial_marker_file=None

    # SQUARE
    def pos_con(self):
        SPEED = 125

        self._robot.setMotors(-420,420,SPEED)
        rospy.sleep(3.5)

        self._robot.setMotors(50,50,SPEED)
        rospy.sleep(1)

    def fwdFast(self):
        SPEED=175
        self._robot.setMotors(-100,-100,SPEED)
        rospy.sleep(1)

    def fwd(self):
        SPEED=100
        self._robot.setMotors(-100,-100,SPEED)
        rospy.sleep(1)

    def back(self):
        SPEED=50
        self._robot.setMotors(50,50,SPEED)
        rospy.sleep(1) 

    def right(self):
        SPEED=50
        self._robot.setMotors(50,-50,SPEED)
        rospy.sleep(1)

    def left(self):
        SPEED=50
        self._robot.setMotors(-50,50,SPEED)
        rospy.sleep(1)

    def turnRight(self):
        SPEED=100
        self._robot.setMotors(220,-220,150)
        rospy.sleep(2.25)

    def turnLeft(self):
        SPEED=100
        self._robot.setMotors(-210,210,SPEED)
        rospy.sleep(2.25)

    def stop(self):
        SPEED=00
        self._robot.setMotors(00,00,SPEED)
        rospy.sleep(1)
        self._robot.setMotors(00,00,SPEED)

    def joy_handler(self, ps):
        self._joystick_buttons =  ps.buttons
        self._joystick_axes = ps.axes



    def spin(self):
        Lft_t = self._joystick_axes[0]
        Lft_d = self._joystick_axes[1]
        Rgh_t = self._joystick_axes[3]
        Rgh_d = self._joystick_axes[4]
        AageL = self._joystick_axes[2]
        AageR = self._joystick_axes[5]
        L_R = self._joystick_axes[6]
        F_B = self._joystick_axes[7]
        sq = self._joystick_buttons[0]
        xx = self._joystick_buttons[1]
        ci = self._joystick_buttons[2]
        tr = self._joystick_buttons[3]
        self._speed_s = self._joystick_buttons[4]
        self._speed_f = self._joystick_buttons[5]
        AageL_Button = self._joystick_buttons[6]
        AageR_Button = self._joystick_buttons[7]
        share = self._joystick_buttons[8]
        options = self._joystick_buttons[9]
        pressL = self._joystick_buttons[10]
        pressR = self._joystick_buttons[11]
        power = self._joystick_buttons[12]
        self._speed -= ((AageR-1)*10)
        self._speed += ((AageL-1)*10)
        self._speed = int(self._speed)
        if (self._speed<0):
            self._speed=0
        elif (self._speed>330):
            self._speed=330
        
        self._speed_set = self._speed

        ll = (Lft_d*self._distance)
        rr = (Rgh_t*self._distance)
        if (rr>=0):
            x = (-ll - rr)
            y = (-ll + rr)
        else:
            x = (-ll - rr)
            y = (-ll + rr) 

        x=int(x)
        y=int(y)

        speeddif = abs(self._speed_ramp - self._speed_set)

        #if the fiducial_marker_file exists, open it
        try:
          #get the filename
          self.fiducial_marker_file_name =self.fiducial_marker_file_path+self.fiducial_marker_file_name
          #get the last line from the file by running tail
          string = subprocess.check_output(['tail','-n1','/home/charisma/processing-3.5.3/output1.txt'])
          #remove line-endings from that last line
          string = string.splitlines()[0]
          #unpack the data in variables
          data = string.split(',')
          #sample line: 42881,3.0,3.0,3.0
          self.fiducial_marker_data['time'] = float(data[0])
          self.fiducial_marker_data['x'] = float(data[1])
          self.fiducial_marker_data['y'] = float(data[2])
          self.fiducial_marker_data['angle'] = float(data[3])
          print("Hey I read %s", string)
          print("Hey we got ", self.fiducial_marker_data)
        except IOError:
          print("Fidcuial marker file " + self.fiducial_marker_file_name + "cannot be opened")


        if (tr == 1) or (ci == 1) or (xx == 1) or (sq == 1):
            x_desired = 1.0
            y_desired = 1.0
            

            x_curr = self.fiducial_marker_data['x']
            y_curr = self.fiducial_marker_data['y']
            ang_curr = self.fiducial_marker_data['angle']

            ang_desired = atan2(y_desired-y_curr,x_desired-x_curr)

            ang_diff = abs(ang_desired-ang_curr)
            x_diff = abs(x_desired-x_curr)
            y_diff = abs(y_desired-y_curr)

            while ang_diff > 0.1:
                SPEED=100
                self._robot.setMotors(220,-220,150)
                rospy.sleep(0.1)

            while (x_diff > 0.1) or (y_diff > 0.1):
                SPEED=100
                self._robot.setMotors(-100,-100,SPEED)
                rospy.sleep(0.1)
                if ang_diff > 0.1:
                    SPEED=0
                    self._robot.setMotors(-100,-100,SPEED)
                    rospy.sleep(0.5)
                    while ang_diff > 0.1:
                        SPEED=100
                        self._robot.setMotors(220,-220,150)
                        rospy.sleep(0.1)
                




        if (self._speed_ramp<self._speed_set):
            self._speed_ramp += (speeddif/20)
        else:
            self._speed_ramp -= (speeddif/20)

        if (self._speed_ramp<0):
            self._speed_ramp=0
        elif (self._speed_ramp>330):
            self._speed_ramp=330

        if (self._speed_set > 150):
            if (0<x<10):
                x=10
                if (self._speed_ramp>150):
                    self._speed_ramp = 150
            elif (-10<x<0):
                x=-10
                if (self._speed_ramp>150):
                    self._speed_ramp = 150

            if (0<y<10):
                y=10
                if (self._speed_ramp>150):
                    self._speed_ramp = 150
            elif (-10<y<0):
                y=-10
                if (self._speed_ramp>150):
                    self._speed_ramp = 150
        else:
            if (0<x<5):
                x=5
            elif (-5<x<0):
                x=-5

            if (0<y<5):
                y=5
            elif (-5<y<0):
                y=-5

        if (self._x_ramp < self._last_x):
            self._x_ramp += 1
        elif (self._x_ramp == self._last_x):
            pass
        else:
            self._x_ramp -= 1

        if (self._y_ramp < self._last_y):
            self._y_ramp += 1
        elif (self._y_ramp == self._last_y):
            pass
        else:
            self._y_ramp -= 1

        if (x==0 and y==0):
            self._speed_ramp -= (self._speed_set/10)
        else:
            if ((abs(self._x_ramp-x)>20) or (abs(self._y_ramp-y)>20)):
                self._speed_ramp = 50
        
        if (self._speed_ramp<0):
            self._speed_ramp=0

        self._last_x = x
        self._last_y = y
        print (self._x_ramp, x, self._last_x, self._y_ramp, y, self._last_y, self._speed_ramp, self._speed_set)
        self._robot.setMotors(self._x_ramp, self._y_ramp, self._speed_ramp)
        self._robot.flushing()






if __name__ == "__main__":    
    robot = NeatoNode()
    pub_led = rospy.Publisher("/led"+chairbot_number, Int8, queue_size=10)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        robot.spin()
        r.sleep()
    # shut down
    self._robot.setLDS("off")
    self._robot.setTestMode("off")
