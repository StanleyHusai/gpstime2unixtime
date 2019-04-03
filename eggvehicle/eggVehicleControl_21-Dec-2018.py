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
#
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

# import puCSV2Topi
#/*send control information
  #*author: WEN Weisong (17902061g@connect.polyu.hk)
  #*date:2018.07.05
  #* sendCarInfoKernel(float steeringAngle,unsigned char steeringTorque, float d, float motionAcc, unsigned char autuatorMode)
  #*detail:callback function
  #* msg->pose.pose.position.x  ->steeringAngle
  #* msg->pose.pose.position.y  ->steeringTorque
  #* msg->pose.pose.position.z  -> reserved
  #* msg->twist.twist.linear.x  -> d
  #* msg->twist.twist.linear.y  -> motionAcc
  #* msg->twist.twist.linear.z  -> autuatorMode
  #* 
  #*/
class eggVehicleControl(QtCore.QThread):
    def __init__(self, parent=None):
	super(eggVehicleControl, self).__init__(parent)
        # rospy.Subscriber('/bounding_boxes',BoundingBoxArray,self.BoundingBoxCallback)
        rospy.Subscriber('/ndt_pose', PoseStamped, self.ndtPoseCallback)
        rospy.Subscriber('/twist_raw', TwistStamped, self.cmdPoseCallback)
        rospy.Subscriber('/estimate_twist', TwistStamped, self.curPoseCallback)
	rospy.Subscriber('/ivactuator', ivactuator, self.ivactuatorCallback)
        rospy.Subscriber('/ndt_reliability', Float32, self.reliabilityCallback)

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
	self.pro_score = 0

	# motion control
	self.prePose = PoseStamped()
	self.curPose = PoseStamped()
	self.curVel = 0.0
	self.objVel = 1.2
	self.velKp = 250
	self.velKd = 350
	
	# waypoint
        self.autuatorMode = 1
        self.motionAcc = 0
	self.objectWayPoint = PoseStamped()
	self.waypointX = []
	self.waypointY = []
	self.waypointZ = []
	self.waypointYaw = []
	self.waypointIndex = 0

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

    def reliabilityCallback(self, data):
	self.pro_score = data.data
	print 'pro_score = ', self.pro_score

    def cmdPoseCallback(self, data):
	self.linear_velocity = data.twist.linear.x
	self.angular_velocity = data.twist.angular.z
        if self.linear_velocity != 0:
	  self.kappa = self.angular_velocity/self.linear_velocity
	self.tire_angle = math.degrees(math.atan(self.wheelbase * self.kappa))
	print 'tire_angle = ', self.tire_angle

    def curPoseCallback(self, data):
	self.current_velocity = data.twist.linear.x

    # def BoundingBoxCallback(self,data):
    #     autuatorMode = []
    #     # BoundingBox human dimension
    #     ppl_dim_x = 0.8 # human size x
    #     ppl_dim_y = 0.8 # human size y
    #     ppl_dim_z = 0.6 # human size z
    #     for i in range(len(data.boxes)):
    #       angles=euler_from_quaternion([data.boxes[i].pose.orientation.x,data.boxes[i].pose.orientation.y,data.boxes[i].pose.orientation.z,data.boxes[i].pose.orientation.w])
    #       x_pp=(data.boxes[i].pose.position.x + data.boxes[i].dimensions.x/2)*math.cos(angles[2])-(data.boxes[i].pose.position.y + data.boxes[i].dimensions.y/2)*math.sin(angles[2])
    #       y_pp=(data.boxes[i].pose.position.x + data.boxes[i].dimensions.x/2)*math.sin(angles[2])+(data.boxes[i].pose.position.y + data.boxes[i].dimensions.y/2)*math.cos(angles[2])
    #       x_pn=(data.boxes[i].pose.position.x + data.boxes[i].dimensions.x/2)*math.cos(angles[2])-(data.boxes[i].pose.position.y - data.boxes[i].dimensions.y/2)*math.sin(angles[2])
    #       y_pn=(data.boxes[i].pose.position.x + data.boxes[i].dimensions.x/2)*math.sin(angles[2])+(data.boxes[i].pose.position.y - data.boxes[i].dimensions.y/2)*math.cos(angles[2])
    #       x_np=(data.boxes[i].pose.position.x - data.boxes[i].dimensions.x/2)*math.cos(angles[2])-(data.boxes[i].pose.position.y + data.boxes[i].dimensions.y/2)*math.sin(angles[2])
    #       y_np=(data.boxes[i].pose.position.x - data.boxes[i].dimensions.x/2)*math.sin(angles[2])+(data.boxes[i].pose.position.y + data.boxes[i].dimensions.y/2)*math.cos(angles[2])
    #       x_nn=(data.boxes[i].pose.position.x - data.boxes[i].dimensions.x/2)*math.cos(angles[2])-(data.boxes[i].pose.position.y - data.boxes[i].dimensions.y/2)*math.sin(angles[2])
    #       y_nn=(data.boxes[i].pose.position.x - data.boxes[i].dimensions.x/2)*math.sin(angles[2])+(data.boxes[i].pose.position.y - data.boxes[i].dimensions.y/2)*math.cos(angles[2])
    #       b_vertices = [(x_pp,y_pp), (x_pn,y_pn),(x_np,y_np),(x_nn,y_nn)]
    #       if (data.boxes[i].dimensions.x < ppl_dim_x and data.boxes[i].dimensions.y < ppl_dim_y and data.boxes[i].dimensions.z > ppl_dim_z and data.boxes[i].dimensions.z < 2.0):
    #         a_vertices = [(-6,-2), (-6,2),(0,-2),(0,2)] # Detection Box Size if human detected
    #       else:
    #         a_vertices = [(-0.8,-1), (-0.8,1),(-0,-1),(-0,1)] # Detection Box Size if no human detected (barrier)
    #       autuatorMode.append(self.separatingaxistheorem(a_vertices, b_vertices))
    #     if (True in autuatorMode):
    #       self.autuatorMode = 0
    #       self.motionAcc = 13000
    #     else:
    #       self.autuatorMode = 1
    #       self.motionAcc = 0

    # def separatingaxistheorem(self,vertices_a, vertices_b):
    #   edges_a = self.vertices_to_edges(vertices_a);
    #   edges_b = self.vertices_to_edges(vertices_b);

    #   edges = edges_a + edges_b

    #   axes = [self.normalize(self.orthogonal(edge)) for edge in edges]

    #   for i in range(len(axes)):
    #     projection_a = self.project(vertices_a, axes[i])
    #     projection_b = self.project(vertices_b, axes[i])
    #     overlapping = self.overlap(projection_a, projection_b)
    #     if not overlapping:
    #         return False
    #   return True

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

    # def normalize(self,v):
    #   from math import sqrt
    #   norm = sqrt(v[0] ** 2 + v[1] ** 2)
    #   return (v[0] / norm, v[1] / norm)

    # def dot(self,a, b):
    #   return a[0] * b[0] + a[1] * b[1];

    # def edge_direction(self,p0, p1):
    #   return (p1[0] - p0[0], p1[1] - p0[1]);

    # def orthogonal(self,v):
    #   return (v[1], -v[0])

    # def vertices_to_edges(self,vertices):
    #   return [self.edge_direction(vertices[i], vertices[(i + 1) % len(vertices)]) \
    #     for i in range(len(vertices))]

    # def project(self,vertices, axis):
    #   dots = [self.dot(vertex, axis) for vertex in vertices]
    #   return [min(dots), max(dots)]

    # def contains(self,n, range_):
    #   a = range_[0]
    #   b = range_[1]
    #   if b < a:
    #     a = range_[1]
    #     b = range_[0]
    #   return (n >= a) and (n <= b);

    # def overlap(self,a, b):
    #   if self.contains(a[0], b):
    #     return True;
    #   if self.contains(a[1], b):
    #     return True;
    #   if self.contains(b[0], a):
    #     return True;
    #   if self.contains(b[1], a):
    #     return True;
    #   return False;

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
