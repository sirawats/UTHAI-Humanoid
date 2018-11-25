#!/usr/bin/env python2

from __future__ import division, print_function
import argparse
import rospy
import copy
from tf.transformations import quaternion_about_axis
from numpy import deg2rad
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Pose2D, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
from std_srvs.srv import Empty
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path, Odometry

parser = argparse.ArgumentParser(description='x1 y1 z1 x2 y2 z2  -->  0 0 0 9 9 90')
parser.add_argument('integers', metavar='N', type=int, nargs='+',
                    help='an integer for the accumulator')
args = parser.parse_args().integers
print(args)


class rviz_footprint:
    """ genarate marker to visualize footstep path """

    def __init__(self, footsteps):
        self.FOOT_VECTOR3 = Vector3(1.5,2.5,0.2)
        self.TEXT_VECTOR3 = Vector3(1,1,1.5)
        self.rviz_footsteps_pub = rospy.Publisher('/footprint', MarkerArray, queue_size=10)
        self.rate = rospy.Rate(10)  # 10h
        self.foots = []
        self.foot = Marker()
        self.foot.header.frame_id = 'map'
        self.foot.type = self.foot.CUBE
        self.foot.ns = 'footstep'
        self.foot.scale = self.FOOT_VECTOR3
        if footsteps == []:
            print('There is impossible path to reach goalself.')
            exit()

        for n in range(len(footsteps)):
            if n == 0:
                self.foot.color = ColorRGBA(0, 1, 1, 0.5)
            elif n == len(footsteps) - 1:
                self.foot.color = ColorRGBA(1, 0, 1, 0.5)

            elif n % 2 == 0:
                self.foot.color = ColorRGBA(0, 1, 0, 0.5)
            else:
                self.foot.color = ColorRGBA(1, 0, 0, 0.5)
            self.foot.pose = footsteps[n].pose
            self.foot.id = n
            self.foots.append(copy.deepcopy(self.foot))
            self.foot.type = self.foot.TEXT_VIEW_FACING
            self.foot.id = n + 1000
            self.foot.text = str(n)
            self.foot.scale = self.TEXT_VECTOR3
            self.foot.color = ColorRGBA(0, 0, 0, 1)
            self.foots.append(copy.deepcopy(self.foot))
            self.foot.type = self.foot.CUBE
            self.foot.scale = self.FOOT_VECTOR3

        self.footstep_path = MarkerArray(self.foots)
        self.foot.action = self.foot.DELETEALL
        self.reset_foot = MarkerArray([copy.deepcopy(self.foot)])

    def clear_footprints(self):
        for i in range(0, 10):
            self.rviz_footsteps_pub.publish(self.reset_foot)
            self.rate.sleep()

    def pub_footprints(self):
        for i in range(0, 20):
            self.rviz_footsteps_pub.publish(self.footstep_path)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('footstep_path')
    rospy.wait_for_service('get_plan')
    pub = rospy.Publisher('/UTHAI/footstep', Pose2D, queue_size=10)
    recieve_plan = rospy.ServiceProxy('get_plan', GetPlan)
    rate = rospy.Rate(10)  # 10h

    """ transformations degree to quaternion """
    Q_s = quaternion_about_axis(deg2rad(args[3]), (0, 0, 1))
    Q_g = quaternion_about_axis(deg2rad(args[7]), (0, 0, 1))
    start = PoseStamped(Header(), Pose(
        Point(args[0], args[1], args[2]), Quaternion(Q_s[0], Q_s[1], Q_s[2], Q_s[3])))
    goal = PoseStamped(Header(), Pose(
        Point(args[4], args[5], args[6]), Quaternion(Q_g[0], Q_g[1], Q_g[2], Q_g[3])))
    result = recieve_plan(start, goal, 0.1)  # call /get_plan service

    footsteps = list(result.plan.poses)

    """rviz display footprints"""
    footprints = rviz_footprint(footsteps)
    footprints.clear_footprints()
    footprints.pub_footprints()

    """ sent footstep viapoint loop"""
    # P = footsteps.pop(0)
    # current_vp = Pose2D(P.pose.position.x, P.pose.position.y, 0)

    # def viapoint_update(data):
    #     global current_vp, footsteps, POL
    #     if footsteps == []:
    #         footsteps = list(result.plan.poses)
    #     P = footsteps.pop(0)
    #     current_vp = Pose2D(P.pose.position.x, P.pose.position.y, 0)
    # s = rospy.Service('next_viapoint', Empty, viapoint_update)
    # while True:
    #     pub.publish(current_vp)
    #     rate.sleep()
