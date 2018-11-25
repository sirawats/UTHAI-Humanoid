#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time
import csv

class uthai_controller:
    def __init__(self):
        self.pb_ankle_pitch_L = rospy.Publisher(
            'uthai/l_ankle_pitch_position/command', Float64, queue_size=10)
        self.pb_ankle_roll_L = rospy.Publisher(
            'uthai/l_ankle_roll_position/command', Float64, queue_size=10)
        self.pb_hip_pitch_L = rospy.Publisher(
            'uthai/l_hip_pitch_position/command', Float64, queue_size=10)
        self.pb_hip_roll_L = rospy.Publisher('uthai/l_hip_roll_position/command',
                            Float64, queue_size=10)
        self.pb_hip_yaw_L = rospy.Publisher('uthai/l_hip_yaw_position/command',
                            Float64, queue_size=10)
        self.pb_knee_pitch_L = rospy.Publisher(
            'uthai/l_knee_pitch_position/command', Float64, queue_size=10)
        self.pb_ankle_pitch_R = rospy.Publisher(
            'uthai/r_ankle_pitch_position/command', Float64, queue_size=10)
        self.pb_ankle_roll_R = rospy.Publisher(
            'uthai/r_ankle_roll_position/command', Float64, queue_size=10)
        self.pb_hip_pitch_R = rospy.Publisher(
            'uthai/r_hip_pitch_position/command', Float64, queue_size=10)
        self.pb_hip_roll_R = rospy.Publisher('uthai/r_hip_roll_position/command',
                            Float64, queue_size=10)
        self.pb_hip_yaw_R = rospy.Publisher('uthai/r_hip_yaw_position/command',
                            Float64, queue_size=10)
        self.pb_knee_pitch_R = rospy.Publisher(
            'uthai/r_knee_pitch_position/command', Float64, queue_size=10)

        '''
        self.hip_yaw_R= self.q_normal[0]
        self.hip_roll_R= self.q_normal[1]
        self.hip_pitch_R= self.q_normal[2]
        self.knee_pitch_R= self.q_normal[3]
        self.ankle_pitch_R= self.q_normal[4]
        self.ankle_roll_R= self.q_normal[5]

        self.hip_yaw_L = self.q_normal[6]
        self.hip_roll_L = self.q_normal[7]
        self.hip_pitch_L = self.q_normal[8]
        self.knee_pitch_L= self.q_normal[9]
        self.ankle_pitch_L= self.q_normal[10]
        self.ankle_roll_L= self.q_normal[11]
        '''
        self.sample = 250  #20   
        self.rate = rospy.Rate(self.sample)
        self.q_normal = [0,0,0,0,0,0,0,0,0,0,0,0]

    def set_pose(self,q):
        for n in range(12):
            if q[n] != 'x':
                self.q_normal[n] += q[n]
    def plus_pose(self,q):
        for n in range(12):
            if q[n] != 'x':
                self.q_normal[n] += q[n]

    def joint_publish(self):
        self.pb_hip_yaw_R.publish(self.q_normal[0])
        self.pb_hip_roll_R.publish(self.q_normal[1])
        self.pb_hip_pitch_R.publish(self.q_normal[2])
        self.pb_knee_pitch_R.publish(self.q_normal[3])
        self.pb_ankle_pitch_R.publish(self.q_normal[4])
        self.pb_ankle_roll_R.publish(self.q_normal[5])

        self.pb_hip_yaw_L.publish(self.q_normal[6])
        self.pb_hip_roll_L.publish(self.q_normal[7])
        self.pb_hip_pitch_L.publish(self.q_normal[8])
        self.pb_knee_pitch_L.publish(self.q_normal[9])
        self.pb_ankle_pitch_L.publish(self.q_normal[10])
        self.pb_ankle_roll_L.publish(self.q_normal[11])

    def move_joint(self,q,t):
        q_plus = []
        for i in range(0,12):
            if q[i] != 'x':
                q_plus += [(q[i]-self.q_normal[i])/(self.sample*t)]
            else:
                q_plus += [0] 
        for n in range(0,int(self.sample*t)):
            self.plus_pose(q_plus)
            self.joint_publish()
            self.rate.sleep()
            

def main():
    rospy.init_node("joint_control_node")
    uthai = uthai_controller()
    time.sleep(1)
    q_down = ['x','x',-0.25,0.7,-0.25,'x']
    q_down += q_down
    q_up = ['x','x',-0.05,0.1,-0.05,'x']
    q_up += q_up

    q_test=[[0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0],
           [0,0.2,-0.1,0,'x','x',0,0.1,-0.8,0.8,'x','x']]
        
    print(q_test[1])
    q_test[1][5] = 0-q_test[1][1]
    q_test[1][11] = 0-q_test[1][7]
    q_test[1][4] = 0-(q_test[1][2]+q_test[1][3])
    q_test[1][10] = 0-(q_test[1][8]+q_test[1][9])
    print(q_test[1])
        #    [0, 0, -0.6283, 1.2566, -0.6283, 0,0, 0, -0.6283, 1.2566, -0.6283, 0]]
        #    [0.0202,-0.0209,0.4500,0.8689,-0.0016,-0.0002,0.0237,-0.3439,0.2873,0.1418,-0.1740,0.1729]]
        #    [0.0058, 0.0620, -0.6297, 1.2826, -0.6357, -0.1816,-0.0054, -0.0559, -0.6258, 1.2484, -0.6289, -0.0006],
        #    [0.0047, 0.0479, -0.6256, 1.2482, -0.6289, 0.0005,-0.0051, -0.0568, -0.6298, 1.2856, -0.6369, 0.1551],
    # uthai.move_joint(q_test[0],1)
    quthai_list = []
    with open('Q.csv', 'rb') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
        for row in spamreader:
            a = row[0].split(',')
            for i in range(len(a)):
                a[i] = float(a[i])
            quthai_list.append(a)
    # with open('qUTHAI1.csv', 'rb') as csvfile:
    #     spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    #     for row in spamreader:
    #         a = row[0].split(',')
    #         for i in range(len(a)):
    #             a[i] = float(a[i])
    #         quthai_list.append(a)

    # with open('qUTHAI2.csv', 'rb') as csvfile:
    #     spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    #     for row in spamreader:
    #         a = row[0].split(',')
    #         for i in range(len(a)):
    #             a[i] = float(a[i])
    #         quthai_list.append(a)
    # with open('qUTHAI3.csv', 'rb') as csvfile:
    #     spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    #     for row in spamreader:
    #         a = row[0].split(',')
    #         for i in range(len(a)):
    #             a[i] = float(a[i])
    #         quthai_list.append(a)
    # while not rospy.is_shutdown():
    uthai.move_joint(quthai_list[0],5)
    for j in range(len(quthai_list)):
        uthai.move_joint(quthai_list[j],0.2)
        print('change',j,quthai_list[j])
        # time.sleep(2)
            

if __name__ == '__main__':
    main()
