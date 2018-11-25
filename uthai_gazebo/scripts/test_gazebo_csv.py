#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time

def main():
    rospy.init_node("q_set_control_node")
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

    with open('Q.csv', 'r') as csvfile:
        spamreader = csvfile.read().split('\n')
        i = 0
        
        for row in spamreader:
            q_set = map(float, row.split(','))
            RHY.publish(q_set[0])
            RHR.publish(q_set[1])
            RHP.publish(q_set[2])
            RKP.publish(q_set[3])
            RAP.publish(q_set[4])
            RAR.publish(q_set[5])

            LHY.publish(q_set[6])
            LHR.publish(q_set[7])
            LHP.publish(q_set[8])
            LKP.publish(q_set[9])
            LAP.publish(q_set[10])
            LAR.publish(q_set[11])
            print(i)
            i = i + 1
            if i==5 :
                time.sleep(5)
            time.sleep(0.2)
    time.sleep(5)
    RHY.publish(0)
    RHR.publish(0)
    RHP.publish(0)
    RKP.publish(0)
    RAP.publish(0)
    RAR.publish(0)
    LHY.publish(0)
    LHR.publish(0)
    LHP.publish(0)
    LKP.publish(0)
    LAP.publish(0)
    LAR.publish(0)

if __name__ == '__main__':
    main()