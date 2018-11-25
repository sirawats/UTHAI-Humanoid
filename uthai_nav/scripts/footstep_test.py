#!/usr/bin/env python2

from __future__ import division, print_function
import argparse
import rospy
import copy
from tf.transformations import quaternion_about_axis
from numpy import deg2rad
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseWithCovarianceStamped


rospy.init_node('pubpoint')
rate = rospy.Rate(20)
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
pub2 = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
init = PoseWithCovarianceStamped()
goal = PoseStamped()
init.header.frame_id = "map"
goal.header.frame_id = "map"

init_q = quaternion_about_axis(deg2rad(90),(0,0,1))
goal_q = quaternion_about_axis(deg2rad(0),(0,0,1))
init.pose.pose.position.x = 90
init.pose.pose.position.y = 40
init.pose.pose.position.z = 0

goal.pose.position.x = 120
goal.pose.position.y = 130
goal.pose.position.z = 0

init.pose.pose.orientation.x = init_q[0]
init.pose.pose.orientation.y = init_q[1]
init.pose.pose.orientation.z = init_q[2]
init.pose.pose.orientation.w = init_q[3]

goal.pose.orientation.x = goal_q[0]
goal.pose.orientation.y = goal_q[1]
goal.pose.orientation.z = goal_q[2]
goal.pose.orientation.w = goal_q[3]

rate.sleep()
pub.publish(init)
rate.sleep()
pub2.publish(goal)
rate.sleep()