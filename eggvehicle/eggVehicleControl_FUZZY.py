#!/usr/bin/env python
#
# Copyright 2018 <copyright holder> <email>
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
#Modified by Hu Sai
#Email: 17095338g@connect.polyu.hk

import rospy
from sensor_msgs.msg import LaserScan
import  math
from matplotlib.patches import Circle
import csv # csv reading needed library
#from nlosExclusion.msg import GNSS_Raw,GNSS_Raw_Array # ros msg
from geometry_msgs.msg import Quaternion, Point, Pose, Twist,PoseStamped, Vector3, TwistStamped # ros message needed
from jsk_recognition_msgs.msg import BoundingBoxArray
from nav_msgs.msg import Odometry, Path 
from PyQt4.QtCore import *
import time
from PyQt4 import QtCore, QtGui
import tf,math
from tf.transformations import euler_from_quaternion
import math
from std_msgs.msg import Float32
from ivactuator.msg import ivactuator
import fuzzy


class eggVehicleControl(QtCore.QThread):
    def __init__(self, parent=None):
	super(eggVehicleControl, self).__init__(parent)
        # rospy.Subscriber('/bounding_boxes',BoundingBoxArray,self.BoundingBoxCallback)
        rospy.Subscriber('/ndt_pose', PoseStamped, self.ndtPoseCallback)
        rospy.Subscriber('/twist_raw', TwistStamped, self.cmdPoseCallback)
        rospy.Subscriber('/estimate_twist', TwistStamped, self.curPoseCallback)
	rospy.Subscriber('/ivactuator', ivactuator, self.ivactuatorCallback)
    	rospy.Subscriber('/fuzzy_kpkd', Odometry, self.fuzzyCallback)


	self.eggVehicleControl_Pub = rospy.Publisher('eggVehicleControl', Odometry,queue_size=100)
	self.control_ = Odometry()
	self.update = 0
	self.preTime = 0
	self.delt = 0
  	self.current_velocity = 0
	self.linear_velocity = 0
	self.angular_velocity = 0
	self.kappa = 0
	self.tire_angle = 0
	self.wheelbase = 1.25
        self.a = []
	self.curYaw = 0

	# motion control
	self.prePose = PoseStamped()
	self.curPose = PoseStamped()
	self.curVel = 0.0
	self.objVel = 1.2
	self.velKp = 0 #250
	self.velKd = 0 #350
	
	# waypoint
        self.autuatorMode = 1
        self.motionAcc = 0
	self.objectWayPoint = PoseStamped()
	self.waypointX = []
	self.waypointY = []
	self.waypointZ = []
	self.waypointYaw = []
	self.waypointIndex = 0

    def fuzzyCallback(self, pd_parameter):
	self.velKp = pd_parameter.pose.pose.position.x
    	self.velKd = pd_parameter.pose.pose.position.y
	print 'kp = ', self.velKp
	print 'kd = ', self.velKd

    def ndtPoseCallback(self, data):  # callback ndt pose
        self.curPose = data # 
	print 'current velocity = ', self.linear_velocity
        self.update = 1
	self.delt = rospy.get_time() - self.preTime
	self.velocityControl()
	self.yawControl()
	self.preTime = rospy.get_time()
	self.prePose = self.curPose

    def ivactuatorCallback(self, data):
	self.curYaw = data.uisteerangle

    def fuzzykpkdCallback(self, data):
	self.fuzzykp = data.data.kp
	print 'fuzzykp = ', self.fuzzykp

    def cmdPoseCallback(self, data):
	self.linear_velocity = data.twist.linear.x
	self.angular_velocity = data.twist.angular.z
        if self.linear_velocity != 0:
	  self.kappa = self.angular_velocity/self.linear_velocity
	self.tire_angle = math.degrees(math.atan(self.wheelbase * self.kappa))
	print 'tire_angle = ', self.tire_angle

    def curPoseCallback(self, data):
	self.current_velocity = data.twist.linear.x

    def run(self):
    	while(1):
    		time.sleep(1)
    		self.eggVehicleControl_Pub.publish(self.control_)
	  
    def velocityControl(self): # motion control 
      self.curVel = self.current_velocity
      deltVel = self.objVel - self.curVel
      if math.fabs(self.control_.pose.pose.position.x) > 400:
	d = 12000
      else:
      	d = 10000
      if self.delt == 0:
	self.delt = 0.00001
      d = math.fabs(d + self.velKp * deltVel + self.velKd * (deltVel/self.delt)) # 0~21500
      self.control_.twist.twist.linear.x  = d
      self.control_.twist.twist.linear.y  = self.motionAcc
      self.control_.twist.twist.linear.z  = self.autuatorMode

    def yawControl(self): # steering control
      steeringAngle = self.tire_angle*14.5
      steeringTorque = 20 + 0.05*abs(self.curYaw - steeringAngle)
      self.control_.pose.pose.position.x = steeringAngle
      self.control_.pose.pose.position.y = steeringTorque
      print 'steering_angle = ', steeringAngle
      print '----------------------------------'

if __name__ == '__main__':
    rospy.init_node('eggVehicleControl', anonymous=True)
    eggVehicleControl_=eggVehicleControl()
    eggVehicleControl_.start()
    print 'process eggVehicleControl...'
    print '--------------------------'
    rate = rospy.Rate(30)  # 30hz
    while not rospy.is_shutdown():
        # rospy.spin()
	rate.sleep()
