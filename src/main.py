#!/usr/bin/env python3
import rospy
import roslib
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import tf
import math
from math import sin, cos, pi,tan, atan2
import numpy as np
from pylab import *
from itertools import groupby
from operator import itemgetter
import matplotlib.pyplot as plt
from scipy import interpolate

from localmap import localmap
pose=[0,0,0]

#responsible for transforming the 
def handle_robot_pose(parent, child, pose):
    br = tf.TransformBroadcaster()
    br.sendTransform((pose[0], pose[1], 0), tf.transformations.quaternion_from_euler(0, 0, pose[2]), rospy.Time.now(), child,parent)

#odometry info
def odometryCb(msg):
    global pose

    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    q0 = msg.pose.pose.orientation.w
    q1 = msg.pose.pose.orientation.x
    q2 = msg.pose.pose.orientation.y
    q3 = msg.pose.pose.orientation.z
    theta=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
    print(f'Odometry pose is {pose}')
    pose=[x,y,theta]

#laser scan info
def scanCb(msg):
    print (pose)
    py,px=[],[]
    scandata=msg.ranges
    angle_min=msg.angle_min
    angle_max=msg.angle_max
    angle_increment=msg.angle_increment
    range_min=msg.range_min
    range_max=msg.range_max
    morigin=[pose[0]/2.0,pose[1]/2.0]
    m=localmap(pose[0], pose[1], 0.05,morigin) 
    m.updatemap(scandata,angle_min,angle_max,angle_increment,range_min,range_max,pose)
    handle_robot_pose("map", "odom", pose)

#publishing into /map    
def mappublisher():
    msg = OccupancyGrid()
    global pose
    while not rospy.is_shutdown():
    	msg.header.frame_id='map'
    	msg.info.resolution = 0.05
    	morigin=[pose[0]/2.0,pose[1]/2.0]
    	m=localmap(pose[0], pose[1], msg.info.resolution,morigin)
    	msg.info.width      = math.ceil(pose[1]/msg.info.resolution)
    	msg.info.height     = math.ceil(pose[0]/msg.info.resolution)
    	msg.info.origin.position.x=-morigin[0]*msg.info.resolution
    	msg.info.origin.position.y=-morigin[1]*msg.info.resolution
    	msg.data = m.localmap
    	mappub.publish(msg)
    	rate.sleep()

    	
if __name__ == "__main__":
    rospy.init_node('main', anonymous=True) #make node
    rospy.Subscriber('/base_odometry/odom', Odometry, odometryCb)
    rospy.Subscriber("/base_scan", LaserScan, scanCb)
    mappub= rospy.Publisher('/map',OccupancyGrid,queue_size=1)
    rate = rospy.Rate(10) # 100hz
    mappublisher()
    
    
    
