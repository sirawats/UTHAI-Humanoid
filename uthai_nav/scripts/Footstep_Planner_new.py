#!/usr/bin/env python2

from __future__ import division, print_function

import rospy
import collections
import heapq
import random
import json
import argparse
import copy
from Tkinter import *
from tf.transformations import quaternion_about_axis, euler_from_quaternion
from numpy import cos, sin,tan, arctan2, deg2rad, rad2deg, square, sqrt,pi
from std_msgs.msg import Float32,Header,ColorRGBA
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, PointStamped,PoseWithCovarianceStamped,Vector3
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from time import sleep, time
from pyexcel_ods import get_data

ROBOT_SIZE = 4
FOOT_SIZE = 2
THETA_LIST = [0, 10, -10]
COM_2_FOOT = [0,0]
UTHAI_between = 0.18
SCALE_REAL = 1.0
SCALE_MAP = 1.0
A_PRECISION = 100

parser = argparse.ArgumentParser(description='footstep_cost.ods')
parser.add_argument('string', metavar='N', type=str, nargs='+',
                    help='ods file name')
args = parser.parse_args().string

def get_footcost_from_excel():
    global SCALE_REAL,UTHAI_between
    # data = get_data("src/Foot_right_cost_test2.ods").popitem()[1]
    data = get_data(args[0]).popitem()[1]
    robot_row = data.pop(0)[0]
    row = data.pop(0)[0]
    normal_line = data.pop(0)[0]
    COM_2_FOOT[0] = row/2
    SCALE_REAL = UTHAI_between/row
    w_param = data.pop(0)
    degree = data.pop(0)
    COST_FOOT_R = {}
    COST_FOOT_L = {}
    THETA_COST_FOOT_R = {}
    THETA_COST_FOOT_L = {}
    for deg in degree:
        COST_FOOT_R[deg] = {}
        for n in range(row):
            row_data = data.pop(0)
            y = robot_row - n - 1
            for x in range(len(row_data)):
                cost = row_data[x]
                if cost != '':
                    COST_FOOT_R[deg][(x, y)] = cost
    "generate footstep cost every possible degree"
    """
    R' = [cos()   sin()]
        [-sin()  cos()]
    """
    for T in range(0, 361, 1):
        THETA_COST_FOOT_R[T] = {}
        THETA_COST_FOOT_L[T] = {}
        for i in COST_FOOT_R.keys():
            COST_FOOT_L[-i] = {}
            THETA_COST_FOOT_R[T][i] = {}
            THETA_COST_FOOT_L[T][-i] = {}
            for k in COST_FOOT_R[i].keys():
                theta_x_r = int(round(cos(deg2rad(T))*k[0] - sin(deg2rad(T)) * k[1]))
                theta_x_l = int(round(cos(deg2rad(T))*(-k[0]) - sin(deg2rad(T)) * k[1]))
                theta_y_r = int(round(sin(deg2rad(T)) * k[0] + cos(deg2rad(T)) * k[1]))
                theta_y_l = int(round(sin(deg2rad(T)) * (-k[0]) + cos(deg2rad(T)) * k[1]))
                    
                '''Linear equation : x + tan(theta)*y - 4 = 0'''
                '''distance : |ax + by +c| / sqrt(a^2 + b^2)'''
                if T != 90 and T != 270 :
                    cost_r = w_param[0]*sqrt(square(theta_x_r) + square(theta_y_r)) + w_param[1]*(abs(theta_x_r + tan(deg2rad(T))*theta_y_r + normal_line)/sqrt(square(theta_x_r) + square(theta_y_r))) 
                    cost_l = w_param[0]*sqrt(square(theta_x_l) + square(theta_y_l)) + w_param[1]*(abs(theta_x_l + tan(deg2rad(T))*theta_y_l + normal_line)/sqrt(square(theta_x_l) + square(theta_y_l))) 
                    THETA_COST_FOOT_R[T][i][(theta_x_r, theta_y_r)] = cost_r + COST_FOOT_R[i][k]
                    THETA_COST_FOOT_L[T][-i][(theta_x_l, theta_y_l) ] = cost_l + COST_FOOT_R[i][k]
                else :
                    cost_r = w_param[0]*sqrt(square(theta_x_r) + square(theta_y_r)) + w_param[1]*theta_y_r
                    cost_l = w_param[0]*sqrt(square(theta_x_l) + square(theta_y_l)) + w_param[1]*theta_y_l
                    THETA_COST_FOOT_R[T][i][(theta_x_r, theta_y_r)] = cost_r + COST_FOOT_R[i][k]
                    THETA_COST_FOOT_L[T][-i][(theta_x_l, theta_y_l) ] = cost_l + COST_FOOT_R[i][k]
    return THETA_COST_FOOT_R, THETA_COST_FOOT_L



THETA_COST_FOOT_R, THETA_COST_FOOT_L = get_footcost_from_excel()
print("scale real world = ",SCALE_REAL)

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


class FootstepGrid(object):
    def __init__(self, MAP):
        self.width = MAP.map.info.width
        self.height = MAP.map.info.height
        self.pose = MAP.map.info.origin
        self.data = list(MAP.map.data)
        self.start = None
        self.goal = None
        self.weights = {}
        self.fcost = {}

    def cost(self, from_node, to_node):
        if to_node[2] == self.goal[2] :
            return self.fcost[to_node] 
        else:
            return self.fcost[to_node] + 0.1 

    def heuristic(self, a, b):
        (x1, y1, theta1, s1) = a
        (x2, y2, theta2, s2) = b
        return sqrt(square(x1 - x2) + square(y1 - y2))

    def point2idx(self, p):
        """
        Convert point (x,y) to index of Map.data
        """
        (x, y, theta, s) = p
        return int((y * self.width) + x)

    def idx2point(self, p):
        """
        Convert index of Map.data to point (x,y)
        """
        return (p % self.width, int(p / self.height), 0)

    def in_bounds(self, id):
        (x, y, theta, s) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id):
        return self.data[self.point2idx(id)] == 0

    def collision(self, id):
        (x, y, theta, s) = id
        results = [(x + ROBOT_SIZE, y, self.goal[2], 1), (x, y - ROBOT_SIZE, self.goal[2], 1), (x - ROBOT_SIZE, y, self.goal[2], 1), (x, y + ROBOT_SIZE, self.goal[2], 1),
                   (x + ROBOT_SIZE, y + ROBOT_SIZE, self.goal[2], 1), (x - ROBOT_SIZE, y + ROBOT_SIZE, self.goal[2], 1), (x + ROBOT_SIZE, y - ROBOT_SIZE, self.goal[2], 1), (x - ROBOT_SIZE, y - ROBOT_SIZE, self.goal[2], 1)]
        for n in results:
            if not self.passable(n):
                return False
        return True

    def foot_collision(self, id):
        (x, y, theta, s) = id
        # results = [(x + FOOT_SIZE, y, self.goal[2], 1), (x, y - FOOT_SIZE, self.goal[2], 1), (x - FOOT_SIZE, y, self.goal[2], 1), (x, y + FOOT_SIZE, self.goal[2], 1),
        #            (x + FOOT_SIZE, y + FOOT_SIZE, self.goal[2], 1), (x - FOOT_SIZE, y + FOOT_SIZE, self.goal[2], 1), (x + FOOT_SIZE, y - FOOT_SIZE, self.goal[2], 1), (x - FOOT_SIZE, y - FOOT_SIZE, self.goal[2], 1)]
        results = [(x + FOOT_SIZE, y + FOOT_SIZE, self.goal[2], 1),
                   (x + FOOT_SIZE, y - FOOT_SIZE, self.goal[2], 1),
                   (x - FOOT_SIZE, y + FOOT_SIZE, self.goal[2], 1),
                   (x - FOOT_SIZE, y + FOOT_SIZE, self.goal[2], 1)]
        for n in results:
            if not self.passable(n):
                return False
        return True

    def square_neighbors(self, id):
        (x, y, theta, s) = id
        # results = [(x + 1, y, self.goal[2], 1), (x, y - 1, self.goal[2], 1), (x - 1, y, self.goal[2], 1), (x, y + 1, self.goal[2], 1),
        #    (x + 1, y + 1, self.goal[2], 1), (x - 1, y + 1, self.goal[2], 1), (x + 1, y - 1, self.goal[2], 1), (x - 1, y - 1, self.goal[2], 1)]
        results = [(x + 1, y, self.goal[2], 1), (x, y - 1, self.goal[2], 1),
                   (x - 1, y, self.goal[2], 1), (x, y + 1, self.goal[2], 1)]
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        # results = filter(self.collision, results)
        return results

    def footstep_neighbors(self, id):
        ''' building the ZMP polygon that foot possible step inside and robot should be stable '''
        (x, y, theta, s) = id
        results = []
        self.fcost = {}
        if s == 0:
            foot_cost = THETA_COST_FOOT_R
        elif s == 1:
            foot_cost = THETA_COST_FOOT_L

        if s == 0:
            new_s = 1
        elif s == 1:
            new_s = 0

        for Tr in THETA_LIST:
            T = (id[2] + Tr)% 360

            cost_current_support = {}

            T_results = []
            for point in foot_cost[T][Tr].keys():
                x_r = x+point[0]
                y_r = y+point[1]
                node = (x_r, y_r, T, new_s)
                T_results.append(node)
                self.fcost[node] = foot_cost[T][Tr][point]

            random.shuffle(T_results)
            results += T_results
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        # results = filter(self.foot_collision, results)
        return results

    def draw_grid(self, id=None):
        for y in range(self.height - 1, -1, -1):
            if y < 10:
                pmap = '{} '.format(y)
            else:
                pmap = '{}'.format(y)
            for x in range(self.width):
                if self.data[self.point2idx((x, y, 0, 1))] == 100:
                    pmap += '+'
                elif (x, y) == (id[0], id[1]):
                    pmap += 'O'
                elif (x, y) == (self.start[0], self.start[1]):
                    pmap += 'A'
                elif (x, y) == (self.goal[0], self.goal[1]):
                    pmap += 'Z'
                else:
                    pmap += '.'
            print(pmap)
        pmap = '  '
        for x in range(self.width):
            pmap += '{}'.format(x % 10)
        print(pmap)


class rviz_footprint:
    """ genarate marker to visualize footstep path """

    def __init__(self, footsteps):
        print("Scale_map = ",SCALE_MAP)
        self.FOOT_VECTOR3 = Vector3(0.04,0.08,0.2)
        self.TEXT_VECTOR3 = Vector3(0.1,0.1,0.1)
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
            if n <= 1:
                self.foot.color = ColorRGBA(0, 1, 1, 0.5)
            elif n >= len(footsteps) - 2:
                self.foot.color = ColorRGBA(1, 0, 1, 0.5)

            elif n % 2 == 0:
                self.foot.color = ColorRGBA(0, 1, 0, 0.5)
            else:
                self.foot.color = ColorRGBA(1, 0, 0, 0.5)
            self.foot.pose = footsteps[n].pose
            self.foot.pose.position.x = footsteps[n].pose.position.x*SCALE_MAP
            self.foot.pose.position.y = footsteps[n].pose.position.y*SCALE_MAP
            # print(self.foot.pose.position.x,self.foot.pose.position.y)
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
def a_star_search(graph, mode='continue'):
    if not(graph.passable(graph.start) and graph.in_bounds(graph.start)):
        
        print('start point is impassable or out of bounds',graph.start)
    if not(graph.passable(graph.goal) and graph.in_bounds(graph.goal)):
        print('goal point is impassable or out of bounds',graph.goal)
    if not(graph.passable(graph.start) and graph.passable(graph.goal) and graph.in_bounds(graph.start) and graph.in_bounds(graph.goal)):
        return {}, {}
    G = {}
    came_from = {}
    open_set = PriorityQueue()

    open_set.put(graph.start, 0)
    came_from[graph.start] = None
    G[graph.start] = 0
    Wd = 0
    Wg = 2
    Wh = 1
    cnt = 0
    lowest_heuristic = None
    while not open_set.empty():
        cnt += 1
        current = open_set.get()
        if current == graph.goal:
            break
        if mode == 'continue':
            neighbors = graph.square_neighbors(current)
        elif mode == 'footstep':
            neighbors = graph.footstep_neighbors(current)
        if cnt % 500 == 0:
            print(cnt, 'len :', len(neighbors))
        for idx in range(len(neighbors)):
            next = neighbors[idx]
            new_cost = G[current] + graph.cost(current, next)  # G(n)
            if next not in G or new_cost < G[next]:
                ''' F(n) = G(n) + H(n) '''
                next_heuristic = graph.heuristic(graph.goal,next) 
                # priority = Wd * (idx + 1) + Wg * new_cost + Wh * graph.heuristic(graph.goal, next)
                priority = Wg * new_cost + Wh * next_heuristic
                if lowest_heuristic == None or (next[2] == graph.goal[2] and next_heuristic < lowest_heuristic) :
                    lowest_heuristic = next_heuristic
                open_set.put(next, priority)
                came_from[next] = current
                G[next] = new_cost
    print('lowest_heuristic =',lowest_heuristic)
    return came_from, G


def reconstruct_path(came_from, start, goal):
    if start not in came_from or goal not in came_from:
        return []
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()  # optional
    return path


def via_point_generate(path):
    """
    path to linear via point generator
    (2, 4), (2, 5), (2, 6), (2, 7), (2, 8), (2, 9), (2, 10), (2, 11), (2, 12), (2, 13)
    |                                                                             |
    V                                                                             |
    (2, 4)  ,   (2, 13) <----------------------------------------------------------
    origin      destination

    """
    vp = list(path)
    i = 0
    while i < len(vp):
        if i + 1 < len(vp):
            if (vp[i][0] - vp[i + 1][0] == 0 and vp[i][0] - vp[i - 1][0] == 0)or \
                    (vp[i][1] - vp[i + 1][1] == 0 and vp[i][1] - vp[i - 1][1] == 0):
                vp.remove(vp[i])
                # if i != len(vp) - 2:
                i -= 1
        i += 1
    return vp


def rviz_map_search(g, scale, delay=0.07, viapoint=True):
    """
    search path and visualize on rviz
    -- set rviz topic as
    1.) /pose           # current cell
    2.) /p_start        # start cell
    3.) /p_goal         # target cell
    """
    def click_listener(data):
        X = int(round(data.point.x - 0.5))
        Y = int(round(data.point.y))
        A = myMap.data[g.point2idx((X, Y, 0, 1))]
        if A == 0:
            myMap.data[g.point2idx((X, Y, 0, 1))] = 100
        else:
            myMap.data[g.point2idx((X, Y, 0, 1))] = 0
        g.data = list(myMap.data)

    rate = rospy.Rate(1 / delay)
    pub = rospy.Publisher('/pose', PoseStamped, queue_size=10)
    pub_start = rospy.Publisher('/p_start', PoseStamped, queue_size=10)
    pub_goal = rospy.Publisher('/p_goal', PoseStamped, queue_size=10)
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    rospy.Subscriber('/clicked_point', PointStamped, click_listener)
    pstamped = PoseStamped()
    pstamped.header.frame_id = 'map'

    pstamped.pose.orientation.x = 0
    pstamped.pose.orientation.y = 0
    pstamped.pose.orientation.z = 0
    pstamped.pose.orientation.w = 1

    pstamped_start = PoseStamped()
    pstamped_start.header.frame_id = 'map'
    pstamped_start.pose.orientation = pstamped.pose.orientation
    pstamped_start.pose.position.x = g.start[0] * scale
    pstamped_start.pose.position.y = g.start[1] * scale

    pstamped_goal = PoseStamped()
    pstamped_goal.header.frame_id = 'map'
    pstamped_goal.pose.orientation = pstamped.pose.orientation
    pstamped_goal.pose.position.x = g.goal[0] * scale
    pstamped_goal.pose.position.y = g.goal[1] * scale

    myMap = OccupancyGrid()
    myMap.header.frame_id = 'map'
    myMap.info.resolution = 1.0
    myMap.info.width = g.width
    myMap.info.height = g.height
    myMap.info.origin = g.pose
    myMap.data = g.data
    came_from, cost_so_far = a_star_search(g, mode='continue')
    path = reconstruct_path(came_from, g.start, g.goal)
    print('path :', path, '  len :', len(path))
    if viapoint:
        vp = via_point_generate(path)
        print('via point :', vp, '  len :', len(vp))
        print('save :', len(path) - len(vp))
        path = vp

    root = Tk()

    global pause
    pause = True

    def key(event):
        global pause
        # print("pressed", event.char)
        if event.char == 's':
            pause = True
            print('pause', pause)
        elif event.char == 'd':
            pause = False
            print('play', pause)

    def callback(event):
        frame.focus_set()

    def sensor_passable(id):
        return g.collision(id) and g.passable(id)

    frame = Frame(root, width=200, height=200)
    frame.bind("<Key>", key)
    frame.bind("<Button-1>", callback)
    frame.pack()
    player = path.pop(0)
    while path != []:
        root.update()
        if pause:
            # print('current position = ', player)
            pass
        else:
            if not sensor_passable(path[0]):
                print('crash !')
                # cost_so_far[player] = 0
                g.start = player
                print('old :', path)
                path = map_search(g, player, g.goal)
                # came_from, cost_so_far = a_star_search(g)
                # path = reconstruct_path(came_from, g.start, g.goal)
                g.draw_grid(player)

            player = path.pop(0)
            pstamped.pose.position.x = player[0] * scale
            pstamped.pose.position.y = player[1] * scale

        pub.publish(pstamped)
        pub_start.publish(pstamped_start)
        pub_goal.publish(pstamped_goal)
        map_pub.publish(myMap)
        rate.sleep()
    return path


def map_search(g, start, goal, mode='continue'):
    """
    path planning with A* Search
    return list of via point
    """
    print('searching...')
    start_time = time()


    g.start = (start[0], start[1], start[2], start[3])
    g.goal = (goal[0], goal[1], goal[2], goal[3])
    came_from, cost_so_far= a_star_search(g, mode)
    if came_from == {} and cost_so_far == {}:
        return []
    path = reconstruct_path(came_from, g.start, g.goal)
    print('elapsed_time : ', time() - start_time)
    print('start :',start)
    print('goal :',goal)
    return path



def nav_service_footstep_planning(g):
    """
    service node for call list of viapoint
    output message is nav_msgs.msg/Path
    Example.
        rosservice call /get_plan [start,goal,tolerance]

    -- start,goal : PoseStamped
    -- tolerance : float32

    """

    def vp_send(req):
        try:
            start_qtn = req.start.pose.orientation
            goal_qtn = req.goal.pose.orientation
            theta_start = euler_from_quaternion(
                [start_qtn.x, start_qtn.y, start_qtn.z, start_qtn.w])
            theta_goal = euler_from_quaternion(
                [goal_qtn.x, goal_qtn.y, goal_qtn.z, goal_qtn.w], axes='sxyz')
            theta_start = int(rad2deg(theta_start[2]))
            theta_goal = int(rad2deg(theta_goal[2]))
            print(theta_start, theta_goal)
        except:
            theta_start = 0
            theta_goal = 0

        start = (int(req.start.pose.position.x), int(
            req.start.pose.position.y), theta_start)
        goal = (int(req.goal.pose.position.x), int(
            req.goal.pose.position.y), theta_goal)
        vp = map_search(g, start, goal, mode='footstep')
        ps_list = []
        while vp != []:
            ''' (x,y) to PoseStamped '''
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            v = vp.pop(0)
            qtn = quaternion_about_axis(deg2rad(v[2]), (0, 0, 1))
            pose.pose.orientation.x = qtn[0]
            pose.pose.orientation.y = qtn[1]
            pose.pose.orientation.z = qtn[2]
            pose.pose.orientation.w = qtn[3]

            pose.pose.position.x = v[0]
            pose.pose.position.y = v[1]
            ps_list.append(pose)
        path = Path()
        path.poses = ps_list
        return path

    s = rospy.Service('get_plan', GetPlan, vp_send)
    rospy.spin()

class nav_spin_footstep_planning:

    def __init__(self,g):
        self.new_start = (0,0,0)
        self.new_goal = (0,0,0)
        self.old_start = (0.0,0)
        self.old_goal = (0,0,0)
        self.sub_start = rospy.Subscriber('/initialpose',PoseWithCovarianceStamped,self.__start)
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.__goal)
        self.pb_path = rospy.Publisher('uthai/footstep_path',Path,queue_size=1)
        self.start_L = (0,0,0,0)
        self.start_R = (0,0,0,1)
        self.goal_L = (0,0,0,0)
        self.goal_R = (0,0,0,1)

        rospy.spin()

    def __searching(self,start,goal):


        Diff_x_start = round(cos(deg2rad(start[2])) * COM_2_FOOT[0] - sin(deg2rad(start[2])) * COM_2_FOOT[1])
        Diff_y_start = round(sin(deg2rad(start[2])) * COM_2_FOOT[0] + cos(deg2rad(start[2])) * COM_2_FOOT[1])
        Diff_x_goal = round(cos(deg2rad(goal[2])) * COM_2_FOOT[0] - sin(deg2rad(goal[2])) * COM_2_FOOT[1])
        Diff_y_goal = round(sin(deg2rad(goal[2])) * COM_2_FOOT[0] + cos(deg2rad(goal[2])) * COM_2_FOOT[1])

        self.start_R = (start[0]+Diff_x_start , start[1] - Diff_y_start , start[2] ,1)
        self.goal_R = (goal[0]+Diff_x_goal , goal[1] - Diff_y_goal , goal[2] ,1)

        vp = map_search(g, self.start_R, self.goal_R, mode='footstep')
        if vp == [] :
            print("error no path possible")
            return 0
        self.start_L = (vp[0][0]-Diff_x_start , vp[0][1] - Diff_y_start , vp[0][2] ,0)
        self.goal_L = (vp[-1][0]-Diff_x_goal , vp[-1][1] - Diff_y_goal , vp[-1][2] ,0)
        
        vp = [self.start_L] + vp + [self.goal_L]
        ps_list = []
        grid2real = []
        seq = 0
        rad_oldstep = 0
        print(vp)
        while vp != []:
            ''' (x,y) to PoseStamped '''
            pose = PoseStamped()
            real = PoseStamped()
            v = vp.pop(0)
            if v[3] == 1:
                pose.header.frame_id = 'r_foot_ft_link'
            elif v[3] == 0:
                pose.header.frame_id = 'l_foot_ft_link'
            qtn = quaternion_about_axis(deg2rad(v[2]), (0, 0, 1))
            pose.pose.orientation.x = qtn[0]
            pose.pose.orientation.y = qtn[1]
            pose.pose.orientation.z = qtn[2]
            pose.pose.orientation.w = qtn[3]
            pose.header.seq = seq
            seq += 1
            real = copy.deepcopy(pose)
            pose.pose.position.x = v[0]
            pose.pose.position.y = v[1]
            ps_list.append(pose)
            if grid2real == []:
                real.pose.position.x = 0
                real.pose.position.y = 0
                grid2real.append(real)
            else :
                real.pose.position.x = -((v[0]*SCALE_REAL) - ps_list[-2].pose.position.x*SCALE_REAL)*sin(rad_oldstep) + ((v[1]*SCALE_REAL) - ps_list[-2].pose.position.y*SCALE_REAL)*cos(rad_oldstep)
                real.pose.position.y = -(((v[0]*SCALE_REAL) - ps_list[-2].pose.position.x*SCALE_REAL)*cos(rad_oldstep) + ((v[1]*SCALE_REAL) - ps_list[-2].pose.position.y*SCALE_REAL)*sin(rad_oldstep))
                grid2real.append(real)
            
            qtn_real = quaternion_about_axis(deg2rad(v[2])-rad_oldstep, (0, 0, 1))
            real.pose.orientation.x = qtn_real[0]
            real.pose.orientation.y = qtn_real[1]
            real.pose.orientation.z = qtn_real[2]
            real.pose.orientation.w = qtn_real[3]
            rad_oldstep = deg2rad(v[2])
        path = Path()
        path_realworld = Path()
        path.poses = ps_list
        footsteps = ps_list
        path_realworld.poses = grid2real
        """rviz display footprints"""
        footprints = rviz_footprint(footsteps)
        footprints.clear_footprints()
        footprints.pub_footprints()

        print('start_L = ',self.start_L)
        print('start_R = ',self.start_R)
        print('goal_L = ',self.goal_L)
        print('goal_R = ',self.goal_R)
        self.pb_path.publish(path_realworld)
        # print(path_realworld)
        
        return path_realworld

    def __start(self,data):

        theta = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,
                                       data.pose.pose.orientation.z,data.pose.pose.orientation.w,])
        theta = int(round(rad2deg(theta[2]+3*pi/2)/10)*10)%360
        self.new_start = (int(round(data.pose.pose.position.x/SCALE_MAP)),int(round(data.pose.pose.position.y/SCALE_MAP)),theta)
        print('self.new_start = ',self.new_start)

    def __goal(self,data):
        theta = euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,
                                       data.pose.orientation.z,data.pose.orientation.w])
        theta = int(round(rad2deg(theta[2]+3*pi/2)/10)*10)%360
        self.new_goal = (int(round(data.pose.position.x/SCALE_MAP)),int(round(data.pose.position.y/SCALE_MAP)),theta)
        print('self.new_goal = ',self.new_goal)
        if self.new_goal != self.old_goal :
            self.__searching(self.new_start,self.new_goal)
        self.old_start = (self.new_start[0],self.new_start[1],self.new_start[2])
        self.old_goal = (self.new_goal[0],self.new_goal[1],self.new_goal[2])



if __name__ == '__main__':
    rospy.init_node('footstep_planner')
    rospy.wait_for_service('static_map')
    print('footstep_planner start !')
    call_map = rospy.ServiceProxy('static_map', GetMap)
    MAP = call_map()
    SCALE_MAP = MAP.map.info.resolution
    g = FootstepGrid(MAP)


    # path = nav_service_footstep_planning(g)
    path = nav_spin_footstep_planning(g)
