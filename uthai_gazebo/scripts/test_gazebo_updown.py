#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time


def main():
    rospy.init_node("joint_control_node")
    LAP = rospy.Publisher(
        'uthai/l_ankle_pitch_position/command', Float64, queue_size=10)
    LAR = rospy.Publisher(
        'uthai/l_ankle_roll_position/command', Float64, queue_size=10)
    LHP = rospy.Publisher(
        'uthai/l_hip_pitch_position/command', Float64, queue_size=10)
    LHR = rospy.Publisher('uthai/l_hip_roll_position/command',
                          Float64, queue_size=10)
    LHY = rospy.Publisher('uthai/l_hip_yaw_position/command',
                          Float64, queue_size=10)
    LKP = rospy.Publisher(
        'uthai/l_knee_pitch_position/command', Float64, queue_size=10)
    RAP = rospy.Publisher(
        'uthai/r_ankle_pitch_position/command', Float64, queue_size=10)
    RAR = rospy.Publisher(
        'uthai/r_ankle_roll_position/command', Float64, queue_size=10)
    RHP = rospy.Publisher(
        'uthai/r_hip_pitch_position/command', Float64, queue_size=10)
    RHR = rospy.Publisher('uthai/r_hip_roll_position/command',
                          Float64, queue_size=10)
    RHY = rospy.Publisher('uthai/r_hip_yaw_position/command',
                          Float64, queue_size=10)
    RKP = rospy.Publisher(
        'uthai/r_knee_pitch_position/command', Float64, queue_size=10)

    q_set = [
        [0, 0, -0.15, 0.3, -0.15, 0, 0, 0, -0.15, 0.3, -0.15, 0],
        [0, 0, -0.6283, 1.2566, -0.6283, 0, 0, 0, -0.6283, 1.2566, -0.6283, 0]
    ]

    def post(q):
        LHY.publish(q[0])
        RHY.publish(q[1])
        LAR.publish(q[2])
        LHR.publish(q[3])
        RAR.publish(q[4])
        RHR.publish(q[5])

        LHP.publish(q[6])
        LKP.publish(q[7])
        LAP.publish(q[8])
        RHP.publish(q[9])
        RKP.publish(q[10])
        RAP.publish(q[11])
        time.sleep(1)
    time.sleep(5)

    while not rospy.is_shutdown():
        time.sleep(5)
        LHP.publish(-0.25)
        LKP.publish(0.5)
        LAP.publish(-0.25)
        RHP.publish(-0.25)
        RKP.publish(0.5)
        RAP.publish(-0.25)
        time.sleep(5)
        LHP.publish(-0.05)
        LKP.publish(0.1)
        LAP.publish(-0.05)
        RHP.publish(-0.05)
        RKP.publish(0.1)
        RAP.publish(-0.05)


if __name__ == '__main__':
    main()
