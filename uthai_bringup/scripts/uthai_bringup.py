#!/usr/bin/env python
import rospy
import actionlib

from dynamixel_controllers.srv import SetSpeed
from std_msgs.msg import Float64, Header
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class Uthai(object):
    def __init__(self):
        rospy.loginfo("Initial uthai_bringup")
        self.joint_names = ['r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee_pitch', 'r_ankle_pitch',
                            'r_ankle_roll', 'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll']
        self.joint_names_rviz = ['r_hip_yaw_joint', 'r_hip_roll_joint', 'r_hip_pitch_joint', 'r_knee_pitch_joint', 'r_ankle_pitch_joint',
                            'r_ankle_roll_joint', 'l_hip_yaw_joint', 'l_hip_roll_joint', 'l_hip_pitch_joint', 'l_knee_pitch_joint', 'l_ankle_pitch_joint', 'l_ankle_roll_joint']
        # rospy.loginfo(self.joint_names)
        self.joint_state = JointState()
        self.joint_state_sub = rospy.Subscriber(
            'uthai_controller/state', FollowJointTrajectoryFeedback, self.joint_state_callback)
        self.joint_state_pub = rospy.Publisher('/uthai/joint_states', JointState, queue_size=10)
        self.joint_state_pub_rviz = rospy.Publisher('/uthai/joint_states_rviz', JointState, queue_size=10)
        self.jta = actionlib.SimpleActionClient(
            '/uthai_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting fot joint trajectory action")
        self.jta.wait_for_server()
        rospy.loginfo("Found joint trajectory action!")

    def joint_state_callback(self, msg):
        self.joint_state.header = msg.header
        self.joint_state.name = msg.joint_names
        self.joint_state.position = msg.actual.positions
        self.joint_state.velocity = msg.actual.velocities
        # self.joint_state_pub.publish(self.joint_state)
        self.joint_state.name = self.joint_names_rviz
        self.joint_state_pub_rviz.publish(self.joint_state)
        # rospy.loginfo(self.joint_state)

    def joint_move(self, angles, duration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points.append(point)
        # rospy.loginfo(goal)
        self.jta.send_goal_and_wait(goal)
        rospy.loginfo("Joint move {}".format(angles))


def main():
    rospy.init_node("uthai_bringup")
    uthai = Uthai()
    uthai.joint_move([0.0]*12, 5.0)
    # uthai.joint_move([0,0,-0.62832,1.2566,-0.62832,0,0,0,-0.62832,1.2566,-0.62832,0],4.0)
    rospy.sleep(2.0)
    # with open('MoveCoML.csv', 'r') as csvfile:
    #     spamreader = csvfile.read().split('\n')
       
    #     for row in spamreader:
    #         q_set = map(float, row.split(','))
    #         uthai.joint_move(q_set, 0.1)

    rospy.spin()


if __name__ == '__main__':
    main()
