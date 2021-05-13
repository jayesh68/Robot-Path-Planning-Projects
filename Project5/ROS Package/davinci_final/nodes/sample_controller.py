
from copy import deepcopy
import math
import random
from threading import Thread, Lock
import sys
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from datetime import datetime #Added this line to measure execution time
import actionlib
import control_msgs.msg
import geometry_msgs.msg
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import sensor_msgs.msg
import tf
import trajectory_msgs.msg
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
import time
import numpy as np
import os
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import tf
from tf2_msgs.msg import TFMessage
import rospy
from std_msgs.msg import Float64
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, pprint
from sympy.matrices import Matrix
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

from gazebo_msgs.msg import LinkState

from geometry_msgs.msg import Twist, Vector3Stamped, Pose
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetLinkState, GetLinkStateRequest
from sensor_msgs.msg import Imu
from mpl_toolkits import mplot3d



import matplotlib.pyplot as plt


joint_lims = {"CamJ1": (None, None),
                      "CamJ2": (-0.3232, 0.5),
                      "CamJ3": (-1.4, .64),
                      "CamJ4": (-.337, .1623)}


# joint_offset = {"CamJ1": 0,          
#                 "CamJ2": 0.48,                  # Currect dir
#                 "CamJ3": -.95,                  # Opposite dir
#                 "CamJ4": .16}

g_joint_states = None
g_positions = None
g_pos1 = None
g_velocities = None
verbose = False

m = lambda inch: 0.0254*inch

l1 = m(6.3)
l2 = m(1.83)
l3 = m(8.6)
l4 = m(23.5)
l5 = m(5.28)
l6 = 0

offset1 = 0
offset2 = 0.48
offset3 = -0.95
offset4 = 0.16

q1, q2, q3, q4, q5, q6 = symbols('q1:7')                                 # joint angles theta
d1, d2, d3, d4, d5, d6 = symbols('d1:7')                                 # link offsets
a0, a1, a2, a3, a4, a5 = symbols('a0:6')                                 # link lengths
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5 = symbols('alpha0:6') # joint twist angles

# DH Table
dh = {alpha0:                     1.57, a0:           0,  d1:    -l1, q1:            q1+offset1,
      alpha1:                       3.14, a1:           0,  d2:     l2, q2:                     -1.57,
      alpha2:                       1.57, a2:          l4,  d3:      0, q3:            q2+offset2,
      alpha3:                     1.57, a3:           0,  d4:     l3, q4:                 0,
      alpha4:                    -1.57, a4:          l5,  d5:      0, q5:            q3+offset3,
      alpha5:                        0, a5:           0,  d6:      q4+offset4,q6:                 0
      } 

class Execute:
    link_name = ''
    link_pose = Pose()

    class Node:
        def __init__(self, x):
            self.x = x
            self.parent = None
            self.cost = 0.0

    def __init__(self, start, goal, rand_area,
                 dista=0.21,
                 path_resolution=.01,
                 goal_sample_rate=200,
                 max_iter=5000,
                 connect_circle_dist=0.5):
        
        self.goal_path = self.Node(goal)
        self.node_list = []
        self.min = []
        self.max = []
        self.min.append(-3);self.max.append(3) #Cam J1
        self.min.append(-0.323);self.max.append(0.5) #Cam J2
        self.min.append(-1.4);self.max.append(0.64) #Cam J3
        self.min.append(-0.337);self.max.append(0.1623)  #CamT4
        
        self.joint_names = ["CamJ1", "CamJ2",
                    "CamJ3", "CamJ4"]
        self.deg = lambda rad: rad*180/np.pi


        self.joint_offset = {"CamJ1": 0,          
                        "CamJ2": 0.48,                  # Currect dir
                        "CamJ3": -.95,                  # Opposite dir
                        "CamJ4": .16}

        self.joint_lims = {"CamJ1": (None, None),
                      "CamJ2": (-0.3232, 0.5),
                      "CamJ3": (-1.4, .64),
                      "CamJ4": (-.337, .1623)}

        self.dista = dista

        # rospy.Subscriber("joint_states", JointState, joint_states_callback, queue_size=1)

       

        # print ( model_info_prox( "davinci_final::CamL4" , "world" ))

        # rospy.Subscriber("gazebo/link_states",  LinkStates, self.joint_states_callback, queue_size = 1)  #remove /?
        # rospy.Subscriber('tf', TFMessage, self.joint_callback)
        self.joint_current = []
        self.joint_state = sensor_msgs.msg.JointState()
        # print(self.joint_state.position)


        # self.q_from_joint_state(self.joint_state)
        # self.goal = [1.10617,0.15,0.322521,0.15]

        ###-------------_###
        self.start = self.Node(start)
        self.end = self.Node(goal)
        self.dimension = len(start)
        self.min_rand = rand_area[-1]
        self.max_rand = rand_area[1]
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        # self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist
   

    def curr_joint_pos(self,joint_state, name):
        if name not in joint_state.name:
            return 0
        i = joint_state.name.index(name)
        return joint_state.position[i]

    def workspace_array(self):
        x_ = np.linspace(-0.2, 1.5, 101)
        y_ = np.linspace(-.8, .9, 101)
        z_ = np.linspace(-.2, 0.9, 101)

        x_mat, y_mat, z_mat = np.meshgrid(x_, y_, z_)
        x = x_mat.flatten()
        y = y_mat.flatten()
        z = z_mat.flatten()
        
        counter = 0
        i = 0
        all_ = []
        dist = lambda x,y,z: np.sqrt(x**2 + y**2 + z**2)
        

        dist = lambda x,y: np.sqrt(x**2 + y**2)
        for i in range(len(x)):
            values = self.calculate_thetas((x[i], y[i], z[i]), "cam")
            if values:
                counter += 1
                all_.append((x[i], y[i], z[i]))
                # all_.append((x[i], y[i], z[i], dist(x[i], y[i], z[i])))

                # print(values)
                # print(dist(x[i], y[i], z[i]), "\n")
        print(counter*100/i, "%\n")
        all_ = np.asarray(all_)
        
        return all_




    def q_val(self):
        my_pub = rospy.Publisher('/get_pose'
                                , Pose
                                , queue_size=10
                                )
        my_pose = Pose()

        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        rospy.wait_for_service('/gazebo/get_link_state')
        # q_array = model_info_prox( "davinci_final::CamL4" , "world" )

        link = GetLinkStateRequest()
        link_name='davinci_final::CamL4'

        res_link_state = get_link_state(link_name, None)
        my_pose = res_link_state.link_state.pose
        my_pub.publish(my_pose)
        print(my_pose.position.x,my_pose.position.y,my_pose.position.z)
        return (my_pose.position.x,my_pose.position.y,my_pose.position.z)
    
    def joint_states_callback(self, data): # data of type JointState
        
        global g_joint_states, g_positions, g_velocities, g_pos1

        # rospy.loginfo(data.position)
        # g_joint_states = data
        # g_positions = data.position
        # g_velocities = data.velocity
        # if len(data.velocity) > 0:
        #     g_vel1 = data.velocity[0]
        # # print(g_positions[0])
        # # print(g_velocities[0])
        # return g_positions

    def compute_simple_timing(self, g_positions, time_per_segment):
        v_list = [np.zeros(7) for i in range(0,len(g_positions))]
        a_list = [np.zeros(7) for i in range(0,len(g_positions))]
        t = [i*time_per_segment for i in range(0,len(g_positions))]
        return v_list, a_list, t
    
    def calculate_thetas(self,coords, arm_str, joint_lims=joint_lims):
    
        # deg = lambda rad: rad*180/np.pi


        # joint_offset = {"CamJ1": 0,          
        #                 "CamJ2": 0.48,                  # Currect dir
        #                 "CamJ3": -.95,                  # Opposite dir
        #                 "CamJ4": .16}

        # joint_lims = {"CamJ1": (None, None),
        #                 "CamJ2": (-0.3232, 0.5),
        #                 "CamJ3": (-1.4, .64),
        #                 "CamJ4": (-.337, .1623)}

        m = lambda inch: 0.0254*inch

        l1 = m(6.3)
        l2 = m(1.83)
        l3 = m(8.6)
        l4 = m(23.5)
        l5 = m(5.28)
        l6 = 0

        R = np.asarray([[0.1276,   -0.9918,         0],
                        [0.2784,    0.0358,   -0.9598],
                        [0.9520,    0.1225,    0.2807]])
        coords = R @ np.asarray(coords).T

        x, y, z = tuple(coords)
        arm_str = arm_str.capitalize()

        try:
            t1 = math.acos(l2**2/math.sqrt(x**2 + y**2)) - math.atan2(x,-y)
            r = math.sqrt(x**2 + y**2 - l2**2)
            t2 = math.atan2(l4,l3) - math.acos((r-l5)/math.sqrt(l3**2 + l4**2))
            t3 = t2 - math.pi/2
            d = z - l1 - math.sqrt(l3**2 + l4**2)*math.sin(math.acos((r-l5)/math.sqrt(l3**2 + l4**2)))

            t1s = t1 - 0
            t2s = t2 - 0.48
            t3s = t3 + 0.95
            ds = d - 0.16

            # print(t1s, t2s, t3s, ds)

            legal_t2 = t2s > self.joint_lims[arm_str + "J2"][0] and t2s < self.joint_lims[arm_str + "J2"][1]
            legal_t3 = t3s > self.joint_lims[arm_str + "J3"][0] and t3s < self.joint_lims[arm_str + "J3"][1]
            legal_d =   ds > self.joint_lims[arm_str + "J4"][0] and  ds < self.joint_lims[arm_str + "J4"][1]

            if legal_t2 and legal_t3 and legal_d:
                return t1s, t2s, t3s, ds
            else:
                return False

        except ValueError:
            pass
        except  ZeroDivisionError:
            pass

        # print("No Legal Configurations!")
        return None
    
    def is_state_valid(self, joint_val):
        #Subscribe to Obstacle gazebo/get_model_state

        # needle_thres = 0.05
        # legal_o1 = joint_val[0] < obst_pos.position.x + needle_thres
        # legal_o2 = joint_val[1] < obst_pos.position.y + needle_thres
        # legal_o3 =   joint_val[2] < obst_pos.position.z + needle_thres

        # if not legal_o1 and not legal_o2 and not legal_o3:
        #     return True


        pass



        # goal = [0.647617,-1.082288,0.622521]
    def caller(self,start, goal, min, max):
        joint_list=self.RRTPlanner(start, goal, min, max)        

    def RRT(self, search_until_max_iter=False):


        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_q = self.get_random_node()
            # print(rnd_q.x[0],rnd_q.x[1],rnd_q.x[2],rnd_q.x[3])
            EE_pos = self.EE_pos(rnd_q.x[0],rnd_q.x[1],rnd_q.x[2],rnd_q.x[3])
            # print(EE_pos[0])
            # while self.calculate_thetas((EE_pos[0],EE_pos[1],EE_pos[2]),'Cam') == False:
            #     rnd_q = self.get_random_node()
            #     EE_pos = self.EE_pos(rnd_q[0],rnd_q[1],rnd_q[2],rnd_q[3])
            

            nearest = self.nearest_fun(self.node_list, rnd_q)
            new_node = self.steer(self.node_list[nearest],rnd_q,self.dista)
            Col_pos = self.EE_pos(new_node.x[0],new_node.x[1],new_node.x[2],new_node.x[3])
            # if self.collision_fun(Col_pos):
            nearest = self.near_fun(new_node)
            new_node = self.parent_fun(new_node, nearest)

            if new_node:
                self.node_list.append(new_node)
                self.rewire(new_node, nearest)

            if (not search_until_max_iter) and new_node:
                newest = self.goal()
                if newest is not None:
                    return self.find_path(newest)

        newest = self.goal()
        if newest is not None:
            return self.find_path(newest)

        return None

    def parent_fun(self, new_node, nearest):
        costs = []
        for i in nearest:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            t_node_pos = self.EE_pos(t_node.x[0],t_node.x[1],t_node.x[2],t_node.x[3])
            if t_node and self.collision_fun(t_node_pos):
                costs.append(self.calc_new_cost(near_node, new_node))
                print(self.calc_new_cost(near_node, new_node))

            else:
                # costs.append(float("inf"))  # the cost of collision node
                costs.append(10000)
        print('costs',costs)
        min_cost = min(costs)

        if min_cost == float("inf") or not nearest:
            return None

        min_ind = nearest[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost

        return new_node

    def goal(self):
        safe_goal_inds = []

        dist_to_goal_list = [self.calc_dist_to_goal(n.x) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.dista]

        for newest in goal_inds:
            t_node = self.steer(self.node_list[newest], self.goal_path)
            t_node_pos = self.EE_pos(t_node.x[0],t_node.x[1],t_node.x[2],t_node.x[3])
            if t_node:
                safe_goal_inds.append(newest)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def calc_dist_to_goal(self, x):
        distance = np.linalg.norm(np.array(x) - np.array(self.end.x))
        return distance

    def near_fun(self, new_node):
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))

        if hasattr(self, 'dista'):
            r = min(r, self.dista)
        dist_list = [np.sum((np.array(node.x) - np.array(new_node.x)) ** 2)
                     for node in self.node_list]
        nearest = [dist_list.index(i) for i in dist_list if i <= r ** 2]
        return nearest

    def rewire(self, new_node, nearest):
        for i in nearest:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            t_node_pos = self.EE_pos(edge_node.x[0],edge_node.x[1],edge_node.x[2],edge_node.x[3])
            improved_cost = near_node.cost > edge_node.cost

            if improved_cost and t_node_pos == True:
                self.node_list[i] = edge_node
                self.enumerate(new_node)

    def calc_new_cost(self, from_node, to_node):
        d = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def enumerate(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.enumerate(node)

    def find_path(self, newest):
        path = [self.end.x]
        node = self.node_list[newest]
        while node.parent is not None:
            path.append(node.x)
            node = node.parent
        path.append(node.x)
        reversed(path)
        print(path)
        return path

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:

            rnd_q = self.Node(np.random.uniform(self.min_rand, self.max_rand, self.dimension))
            # print(rnd_q.x)
        # else:  
        #     rnd_q = self.Node(self.end.x)
        else:  # goal point sampling
            rnd = self.Node(self.end.x)
        return rnd

    def steer(self, from_node, to_node, cost_len=float("inf")):
        new_node = self.Node(list(from_node.x))
        d = self.calc_distance_and_angle(new_node, to_node)
        new_node.path_x = [list(new_node.x)]

        if cost_len > d:
            cost_len = d

        n_expand = math.floor(cost_len / self.path_resolution)

        start, end = np.array(from_node.x), np.array(to_node.x)
        v = end - start

        u = v / (np.sqrt(np.sum(v ** 2)))

        print("v",v)
        print("u",u)
        
        for blah in range(n_expand):
            new_node.x += u * self.path_resolution
            new_node.path_x.append(list(new_node.x))

        d = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(list(to_node.x))

        new_node.parent = from_node

        return new_node
    
    def timer_callback(self,event): # Type rospy.TimerEvent
        print("Worked")

    def joint_callback(self,msg): # data of type JointState

        x = msg.transforms[3].transform.translation.x
        y = msg.transforms[3].transform.translation.y
        z = msg.transforms[3].transform.translation.z
        rospy.loginfo('x: {}, y: {}, z: {}'.format(x,y,z))
        return x,y,z

    # def joint_logger_node(self):
        # rospy.init_node('joint_logger_node', anonymous=True)
        # rospy.Subscriber('tf', TFMessage, self.joint_callback)
        # rospy.Subscriber("gazebo/link_states",  JointState, self.joint_callback, queue_size = 1)  #remove /?

        # rospy.Timer(rospy.Duration(2), self.timer_callback)
        # tform = getTransform(tftree,'base_link','camera_link',desiredTime);

    @staticmethod
    def TF_Mat(alpha,a,d,q):
        m = lambda inch: 0.0254*inch

        l1 = m(6.3)
        l2 = m(1.83)
        l3 = m(8.6)
        l4 = m(23.5)
        l5 = m(5.28)

        offset1=0.48
        offset2=-0.95
        offset3=0.16
        TF = Matrix([[ cos(q),   -sin(q)*cos(alpha),   sin(q)*sin(alpha),       a*cos(q)],
                     [ sin(q),    cos(q)*cos(alpha),  -cos(q)*sin(alpha),       a*sin(q)],
                     [      0,           sin(alpha),          cos(alpha),              d],
                     [      0,                    0,                   0,             1]])
        return TF

    def EE_pos(self,q_1,q_2,q_3,q_4):

        m = lambda inch: 0.0254*inch

        l1 = m(6.3)
        l2 = m(1.83)
        l3 = m(8.6)
        l4 = m(23.5)
        l5 = m(5.28)

        offset1=0.48
        offset2=-0.95
        offset3=0.16
        T0_1 = self.TF_Mat(1.57, 0, -l1, q_1+offset1)
        T1_2 = self.TF_Mat(3.14, 0, l2, -1.57)
        T2_3 = self.TF_Mat(1.57, l4, 0, q_2+offset2)
        T3_4 = self.TF_Mat(1.57, 0, l3, 0)
        T4_5 = self.TF_Mat(-1.57, l5, 0, q_3+offset3)
        T5_6 = self.TF_Mat(0, 0, q_4+offset3, 0)
        
        T0_6 = (T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 )
        
        #end effector position


        EE_pos = [T0_6[3],T0_6[7],T0_6[11]]

        return EE_pos

            
    def talker(self,q_disc):

        pub = rospy.Publisher('/davinci_final/CamT1/command', Float64, queue_size=10)
        pub1 = rospy.Publisher('/davinci_final/CamT2/command', Float64, queue_size=10)
        pub2 = rospy.Publisher('/davinci_final/CamT3/command', Float64, queue_size=10)
        pub3 = rospy.Publisher('/davinci_final/CamT4/command', Float64, queue_size=10)

# 0.647617,-0.23,0.622521,0.07
        # pub.publish(0.647617)
        # pub1.publish(-0.23)
        # pub2.publish(0.622521)
        # pub3.publish(0.07)
        pub4 = rospy.Publisher('/davinci_final/GripT1/command', Float64, queue_size=10)
        pub5 = rospy.Publisher('/davinci_final/GripT2/command', Float64, queue_size=10)
        pub6 = rospy.Publisher('/davinci_final/GripT3/command', Float64, queue_size=10)
        pub7 = rospy.Publisher('/davinci_final/GripT4/command', Float64, queue_size=10)
        pub8 = rospy.Publisher('/davinci_final/GripperT1/command', Float64, queue_size=10)
    

        # rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
 
            pub.publish(0.617)
            pub1.publish(-0.35)
            pub2.publish(-0.022521)
            pub3.publish(-0.15)
            print("Got here")
            for i in range(len(q_disc)):
                pub4.publish(q_disc[i][0])
                pub5.publish(q_disc[i][1])
                pub6.publish(q_disc[i][2])
                pub7.publish(q_disc[i][3])
                time.sleep(2.3)
                if i == len(q_disc):
                    quit()

            # if now > 5: 
            #     break
            #     pub1.publish(msg_data2)
            #     # pub2.publish(msg_data2)
            #     # pub3.publish(msg_data2)


    @staticmethod
    def nearest_fun(node_list, rnd_node):
        dlist = [np.sum((np.array(node.x) - np.array(rnd_node.x))**2)
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind



    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x[0] - from_node.x[0]
        dy = to_node.x[1] - from_node.x[1]
        dz = to_node.x[2] - from_node.x[2]
        d = np.sqrt(np.sum((np.array(to_node.x) - np.array(from_node.x))**2))
        return d

    @staticmethod
    def collision_fun(node):

        if node is None:
            return False
        
        dist = sqrt((1.043117-node[0])**2+(-1.21341-node[1])**2+(0.885659-node[2])**2)
        if dist <= 0.15:
            return False  # collision

        return True  # safe

def executer():


    start = [0,0,0,0]
    # print(start)
    end = [0.42, 0.12, 0.29, 0.35]
    # print(end)

    rrt_star = Execute(start=start,
                       goal=end,
                       rand_area=[0, 0.01],
                       max_iter=8000)

    path = rrt_star.RRT(search_until_max_iter=False)

    print('path is',path)
    # path = path[1:len(path)-1]
    path = path[::-1]
    print(path)
    rrt_star.talker(path)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    # ma = Execute()
    executer()    

    rospy.spin()


# #1 - Fwd kinematics Test

# #2 - IK Test: 


# Joint parameters: 0.22867646378864692 0.5925882455669071 0.4517919187720105 -0.31473629905774875
# Expected Gripper Position: [ 0.7438616 -0.1753504  0.0986272]

