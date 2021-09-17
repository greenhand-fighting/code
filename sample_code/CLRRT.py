"""
CC-RRT Complete Edition

Algorithm Ref: Chance Constrained RRT fot Probabilistic Robustness to Environment Uncertainty

set all angle as rad
no control noise
"""
import matplotlib.patches as mpatches
from matplotlib.legend_handler import HandlerPatch
import math
import os
import random
import sys
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.special import erf, erfinv

show_animation = True


class Vehicle:
    def __init__(self,lf,lr,w):
        self.l_f = lf # 前距离
        self.l_r = lr # 后距离
        self.w = w  # 车宽


class ClrrtGo:
    """
    Class for CCRRT planning
    """

    class Node:
        def __init__(self, x, y, yaw):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.conv = np.zeros((3, 3))  # uncertainty matrix
            self.parent = None
            self.time = 0.0  # travaling time, for calculate cost
            self.cc = 0.0  # chance constraint, for calculate cost
            self.cost = 0.0  # cost = f(time, cc)
            self.cost_lb = 0.0  # cost lower bound
            self.cost_ub = math.inf  # cost upper bound

    def __init__(self, car, start, goal, ganzhi_kuang, ganzhi_tuoyuan,rand_area,rengong_tuoyuan):
        """
        Setting Parameter
        start:Start Position [x,y,yaw]
        goal:Goal Position [x,y,yaw]
        obstacleList:obstacle
        randArea:Random Sampling Area [min,max]
        """
        self.car = car
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.ganzhi_kuang = ganzhi_kuang   # 感知框，用于碰撞检测的
        self.ganzhi_tuoyuan = ganzhi_tuoyuan   # 感知椭圆，用于计算机会约束的
        self.rengong_tuoyuan=rengong_tuoyuan # 人工椭圆，并不使用，只是和ccrrt的类形式保持一致

        assert len(rand_area) == 4, "rand_area = [x-min, x-max, y-min, y-max]"
        self.min_rand_x = rand_area[0]
        self.max_rand_x = rand_area[1]
        self.min_rand_y = rand_area[2]
        self.max_rand_y = rand_area[3]

        # self.path_resolution = 1.0
        # self.goal_sample_rate = 10 # goal_sample_rate% set goal as sampling node

        self.node_list = []
        self.max_iter = 80

        self.max_n_path = 20  # save no more than n_path feasible path to choose
        self.n_path_when_change_strategy = 25
        self.max_n_node = 5000  # save no more than max_node nodes on tree
        self.delta_time = 0.1  # dt second
        self.dis_threshold = 1.0  # distance threshold for steering

        self.max_vehicle_turn_rate = np.pi  # vehicle max turning rate
        self.max_angle_diff = np.pi / 2.0
        self.max_vehicle_speed = 18.0  # m/s
        self.min_vehicle_speed = 0.0  # m/s

        self.expect_speed = self.max_vehicle_speed / 2.0  # used for hueristic distance calculation
        self.expect_turn_rate = self.max_vehicle_turn_rate / 4.0  # used for hueristic distance calculation

        # params
        self.k_cc = 100  # for chance_constrain value: node_cost = time + k_cc * node.cc
        # for expect dis: heu_value = k_dis * dis() + (1-k_dis) * node_cost
        # or with k_dis percentage sampling dis() and (1-k_dis) sampling node_cost
        # k_node_cost = 1 - k_dis
        self.k_dis_explore = 0.7  # k_dis value when pay more attention to exploration (more radical)
        self.k_dis_exploit = 0.3  # k_dis value when pay more attention to exploit (safer)
        # the same as k_dis / k_node_cost, but used for find a path relatively close to goal
        self.k_dis_when_no_path = 0.85

        # PID control matrix
        self.P = np.diag([1.0, 5.0])  # 2*2
        # self.I = np.diag([0.1, 0.5]) # 2*2
        self.D = np.diag([-2.0, -6.5])  # 2*2
        self.max_steer_step = 30  # max n_step for steering

        self.p_safe = 0.96  # p_safe for chance constraint#-------------------------------------------------------------------------------这是对完全系数

        self.sigma_x0 = np.diag([0.2, 0.2, 0.1])  # sigma_x0
        self.sigma_control = np.diag([0.0, 0.0])  # control noise
        self.sigma_pose = np.array([
            [0.02, 0.01, 0.00],
            [0.01, 0.02, 0.00],
            [0.00, 0.00, 0.01]
        ])

        self.nearest_node_step = 8  # get nodes to do tree expanding, used in get_nearest_node_index
        self.n_nearest = 15  # get n nearest nodes, used in get_nearest_node_index
        self.steer_back_step = 8  # used after find a path and try connect to goal after steering

        # init
        # self.sigma_x0[0,0] /= self.path_resolution
        # self.sigma_x0[1,1] /= self.path_resolution
        # self.sigma_control[0,0] /= self.path_resolution
        # self.dis_threshold /= self.path_resolution
        # self.max_vehicle_speed /= self.path_resolution
        # self.min_vehicle_speed /= self.path_resolution

        self.start.conv = self.sigma_x0
        self.start.time = 0.0
        self.start.cc = self.get_chance_constrain(self.start)
        self.start.cost = self.get_cost(self.start.time, self.start.cc)
        self.start.cost_lb = self.get_cost_lb(self.start)
        self.start.cost_ub = math.inf

        self.path_end = []  # save path ending point, if len(path_end) >= n_path, we will stop searching to get the best path
        self.path = []  # save the final path

        self.with_metric = False  # planning metric item

    def planning(self, animation=False, with_metric=False):
        """
        cc_rrt path planning
        animation: flag for animation on or off
        with_metric start metric
        """
        print("Begin CL-RRT")
        self.node_list = [self.start]

        if with_metric:
            self.with_metric = True
            self.node_when_find_first_path = -1
            self.time_when_find_first_path = -1.0
            self.total_time = 0.0
            self.timer_start = time.clock()

        for i in range(self.max_iter):
           # if i % 10 == 0:
           #     print("Iter:", i, ", number of nodes:", len(self.node_list))

            sample_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, sample_node, self.n_nearest,
                                                      self.nearest_node_step)

            for idx in nearest_ind:
                nearest_node = self.node_list[idx]

                # Running some checks in increasing order of computational complexity
                # Do some checking
                # Check 1: Does the sample co-inside with nearest node?
                if self.calc_distance(sample_node, nearest_node) < self.dis_threshold:
                    continue
                sample_node.yaw = np.arctan2(
                    sample_node.y - nearest_node.y,
                    sample_node.x - nearest_node.x,
                )

                # Check 2: Is the steer angle within the acceptable range?
                if not self.angle_check(nearest_node, sample_node, self.max_angle_diff):
                    continue

                # local planner sampling (also updates sample.conv)
                self.local_planner(nearest_node, sample_node)

                if len(self.path_end) > self.max_n_path or len(self.node_list) > self.max_n_node:
                    break

            # to displaying cc-rrt searching
            # if i % 5 == 0:
            #     self.draw_graph(sample_node)
            #     plt.pause(2.0)
            # self.draw_graph(sample_node)
            # plt.pause(1.0)

            # if len(path_end) >= n_path, we will stop searching to get the best path
            if len(self.path_end) > self.max_n_path or len(self.node_list) > self.max_n_node:
                break

        # end cc_rrt loop
      #  print("Tree with %d nodes generated" % len(self.node_list))

        if len(self.path_end):
           # print("%d path be found" % len(self.path_end))
            # choosing best path
            min_upper_bound = math.inf
            final_goal_node_idx = -1
            for idx, node in enumerate(self.path_end):
                # max
                path_max_upper_bound = -math.inf
                while node:
                    path_max_upper_bound = max(path_max_upper_bound, node.cost_ub)
                    node = node.parent
                if path_max_upper_bound < min_upper_bound:
                    min_upper_bound = path_max_upper_bound
                    final_goal_node_idx = idx
                # sum
                # path_sum_upper_bound = 0.0
                # while node:
                #     path_sum_upper_bound += node.cost_ub
                #     node = node.parent
                # if path_sum_upper_bound < min_upper_bound:
                #     min_upper_bound = path_sum_upper_bound
                #     final_goal_node_idx = idx
            final_goal_node = self.path_end[final_goal_node_idx]
        else:
           # print("No path found!")
            # choose a feasible path can drive to goal closer
            nearest_ind = self.get_close_to_goal_index(self.node_list)
            final_goal_node = self.node_list[nearest_ind]

        # return path
        while final_goal_node:
            self.path.append(final_goal_node)
            final_goal_node = final_goal_node.parent

        if self.with_metric:
            self.total_time = time.clock() - self.timer_start

        return self.path

    def steer(self, from_node, to_node):
        """
        steer with chance constrain checking
        begin: from_node
        return path = [inter_node, ..., inter_node, to_node(if feasible)]
        """
        # reference v & w
        dis, angle = self.calc_distance_and_angle(from_node, to_node)
        angle = self.angle_wrap(angle - from_node.yaw)

        if abs(angle) > math.pi / 3.0:
            self.P[0, 0] = 0.05
            self.D[0, 0] = -0.10
            self.min_vehicle_speed = 1.0
            self.max_vehicle_turn_rate = math.pi
        elif abs(angle) > math.pi / 6.0:
            self.P[0, 0] = 0.25
            self.D[0, 0] = -0.5
            self.min_vehicle_speed = 6.5
            self.max_vehicle_turn_rate = math.pi
        else:
            self.P[0, 0] = 1.0
            self.D[0, 0] = -2.0
            self.min_vehicle_speed = 13.0
            self.max_vehicle_turn_rate = math.pi / 2.0

        u_p = self.P.dot(np.array([[dis], [angle]]))
        u_d = np.zeros((2, 1))
        u = u_p + u_d
        u[0, 0] = max(self.min_vehicle_speed, min(u[0, 0], self.max_vehicle_speed))
        if abs(u[1, 0]) > self.max_vehicle_turn_rate:
            u[1, 0] = np.sign(angle) * self.max_vehicle_turn_rate

        # prev node: deep copy from the from_node
        prev = self.Node(from_node.x, from_node.y, from_node.yaw)  # local init
        prev.conv = from_node.conv
        prev.cost = from_node.cost
        prev.parent = from_node.parent
        prev.time = from_node.time
        prev.cc = from_node.cc
        prev.cost_lb = from_node.cost_lb
        prev.cost_ub = from_node.cost_ub

        prev_dis = dis
        prev_angle = angle

        J1 = np.diag([1.0, 1.0, 1.0])

        J2 = np.zeros((3, 2))
        J2[0, 0] = self.delta_time * math.cos(prev.yaw)
        J2[1, 0] = self.delta_time * math.sin(prev.yaw)
        J2[2, 1] = self.delta_time

        # get feasible node from N_near->N_sample
        feasible_node_list = []
        n_step = 0
        # feaisble_to_end = True
        while self.calc_distance(prev, to_node) > self.dis_threshold and n_step < self.max_steer_step:
            pose = J1.dot(np.array([[prev.x], [prev.y], [prev.yaw]])) + J2.dot(u)
            inter_node = self.Node(pose[0].item(), pose[1].item(), pose[2].item())
            inter_node.parent = prev
            inter_node.conv = J1.dot(prev.conv).dot(J1.transpose()) + \
                              J2.dot(self.sigma_control).dot(J2.transpose()) + \
                              self.sigma_pose
            inter_node.cc = self.get_chance_constrain(inter_node)
            #if inter_node.cc < 1.0  and self.in_place(inter_node) and self.safe_steer(inter_node):  #--------------------------------------------------------------这里修改了
            if self.in_place(inter_node) and self.safe_steer(inter_node):
                inter_node.time = prev.time + self.delta_time
                inter_node.cost = self.get_cost(inter_node.time, inter_node.cc)
                inter_node.cost_lb = self.get_cost_lb(inter_node)
                feasible_node_list.append(inter_node)
                prev = inter_node  # inter_node will point to the next inter_node

                # update J1 J2
                J2[0, 0] = self.delta_time * math.cos(prev.yaw)
                J2[1, 0] = self.delta_time * math.sin(prev.yaw)
                J2[2, 1] = self.delta_time

                dis, angle = self.calc_distance_and_angle(prev, to_node)
                angle = self.angle_wrap(angle - prev.yaw)

                if abs(angle) > math.pi / 3.0:
                    self.P[0, 0] = 0.05
                    self.D[0, 0] = -0.10
                    self.min_vehicle_speed = 1.0
                    self.max_vehicle_turn_rate = math.pi
                elif abs(angle) > math.pi / 6.0:
                    self.P[0, 0] = 0.25
                    self.D[0, 0] = -0.5
                    self.min_vehicle_speed = 6.5
                    self.max_vehicle_turn_rate = math.pi
                else:
                    self.P[0, 0] = 1.0
                    self.D[0, 0] = -2.0
                    self.min_vehicle_speed = 13.0
                    self.max_vehicle_turn_rate = math.pi / 2.0

                u_p = self.P.dot(np.array([[dis], [angle]]))
                u_d = self.D.dot(np.array([[dis - prev_dis], [angle - prev_angle]]))
                u = u_p + u_d
                u[0, 0] = max(self.min_vehicle_speed, min(u[0, 0], self.max_vehicle_speed))
                if abs(u[1, 0]) > self.max_vehicle_turn_rate:
                    u[1, 0] = np.sign(angle) * self.max_vehicle_turn_rate
                prev_dis = dis
                prev_angle = angle
                n_step += 1
            else:
                # feaisble_to_end = False
                break

        # add to_node if possible
        # if feaisble_to_end and n_step < self.max_steer_step:
        #     to_node.parent = prev
        #     to_node.conv = J1.dot(prev.conv).dot(J1.transpose()) + \
        #                    J2.dot(self.sigma_control).dot(J2.transpose()) + \
        #                    self.sigma_pose
        #     to_node.cc = self.get_chance_constrain(to_node)
        #     if to_node.cc < 1 - self.p_safe:
        #         to_node.time = prev.time + self.delta_time
        #         to_node.cost = self.get_cost(to_node.time, to_node.cc)
        #         to_node.cost_lb = self.get_cost_lb(to_node)
        #         feasible_node_list.append(to_node)

        return feasible_node_list

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def local_planner(self, parent, sample):
        feasible_node_list = self.steer(parent, sample)
        for node in feasible_node_list:
            self.node_list.append(node)  # add to tree
        # find a path to goal
        if len(feasible_node_list) and self.calc_distance(feasible_node_list[-1], self.end) < self.dis_threshold:
            self.path_end.append(feasible_node_list[-1])  # save the end node of the path

            # metric
            if self.with_metric and len(self.path_end) == 1:
                self.time_when_find_first_path = time.clock() - self.timer_start
                self.node_when_find_first_path = len(self.node_list)

            # back propogation
            self.backpropogation(feasible_node_list[-1])
            return
        # no path to goal
        # for each feasible node
        for idx in range(0, len(feasible_node_list), self.steer_back_step):
            # try connecting node to goal
            node = feasible_node_list[idx]
            tmp_end_node = self.Node(self.end.x, self.end.y, 0.0)
            tmp_end_node.yaw = np.arctan2(
                tmp_end_node.y - node.y,
                tmp_end_node.x - node.x,
            )
            if not self.angle_check(node, tmp_end_node, self.max_angle_diff):
                continue
            node_to_goal_list = self.steer(node, tmp_end_node)
            if len(node_to_goal_list) and self.calc_distance(node_to_goal_list[-1],
                                                             self.end) < self.dis_threshold:  # get to goal from current node
                for node in node_to_goal_list:
                    self.node_list.append(node)  # add to tree
                self.path_end.append(node_to_goal_list[-1])  # save the end node of the path

                # metric
                if self.with_metric and len(self.path_end) == 1:
                    self.time_when_find_first_path = time.clock() - self.timer_start
                    self.node_when_find_first_path = len(self.node_list)

                # update upper-bound cost-to-goal of those nodes
                self.backpropogation(node_to_goal_list[-1])
            if len(self.path_end) > self.max_n_path or len(self.node_list) > self.max_n_node:
                break

    def vehicle_constraints(self, x, y, yaw):
        """
        calculate vehicle's edge constraints
        return ([a_1, a_2, a_3, a_4], [b1, b2, b3, b4])
        a_i is the unit outward normals of line constraint 单位外法向量
        b_i = a_i^T * x (x is on the line)
        """

        w = self.car.w / 2.0
        p0 = [
            x + self.car.l_f * math.cos(yaw) + w * math.sin(yaw),
            y + self.car.l_f * math.sin(yaw) - w * math.cos(yaw)
        ]
        p1 = [
            x + self.car.l_f * math.cos(yaw) - w * math.sin(yaw),
            y + self.car.l_f * math.sin(yaw) + w * math.cos(yaw)
        ]
        p2 = [
            x - self.car.l_r * math.cos(yaw) - w * math.sin(yaw),
            y - self.car.l_r * math.sin(yaw) + w * math.cos(yaw)
        ]
        p3 = [
            x - self.car.l_r * math.cos(yaw) + w * math.sin(yaw),
            y - self.car.l_r * math.sin(yaw) - w * math.cos(yaw)
        ]


        a1 = [p0[0] - p3[0], p0[1] - p3[1], 0.0]  # 单位外法向量
        d = math.sqrt(a1[0] ** 2 + a1[1] ** 2)
        a1 = [i / d for i in a1]
        b1 = a1[0] * p0[0] + a1[1] * p0[1]

        a2 = [p1[0] - p0[0], p1[1] - p0[1], 0.0]  # 单位外法向量
        d = math.sqrt(a2[0] ** 2 + a2[1] ** 2)
        a2 = [i / d for i in a2]
        b2 = a2[0] * p1[0] + a2[1] * p1[1]

        a3 = [p2[0] - p1[0], p2[1] - p1[1], 0.0]  # 单位外法向量
        d = math.sqrt(a3[0] ** 2 + a3[1] ** 2)
        a3 = [i / d for i in a3]
        b3 = a3[0] * p2[0] + a3[1] * p2[1]

        a4 = [p3[0] - p2[0], p3[1] - p2[1], 0.0]  # 单位外法向量
        d = math.sqrt(a4[0] ** 2 + a4[1] ** 2)
        a4 = [i / d for i in a4]
        b4 = a4[0] * p3[0] + a4[1] * p3[1]

        return ([a1, a2, a3, a4], [b1, b2, b3, b4])

    def get_chance_constrain(self, current):
        A, B = self.vehicle_constraints(current.x, current.y, current.yaw)
        delta_t = 0  # sum(min delte_tj)
        # cal for each obs
        for obs in self.ganzhi_tuoyuan:
            angle = abs(obs[4])
            angle = angle if angle <= math.pi / 2.0 else math.pi - angle

            delta_tj = math.inf
            for a, b in zip(A, B):
                a = np.array([a])  # 1*3
                x = np.array([[obs[0]], [obs[1]], [0.0]])  # 3*1

                # abs_mat = np.diag([obs[3] * math.sin(angle) + obs[2] * math.cos(angle),
                #                    obs[2] * math.sin(angle) + obs[3] * math.cos(angle), obs[4]])
                abs_mat = np.diag([obs[6] * math.sin(angle) + obs[5] * math.cos(angle),
                                   obs[5] * math.sin(angle) + obs[6] * math.cos(angle), obs[4]])
                sigma = current.conv + abs_mat
                # sigma = current.conv + np.diag([obs[2], obs[3], obs[4]]) # 3*3
                erf_item = (a.dot(x).item() - b) / np.sqrt(2 * a.dot(sigma).dot(a.transpose()).item())
                cc = 0.5 * (1 - erf(erf_item))
                if cc < delta_tj:
                    delta_tj = cc
            delta_t += delta_tj
        return delta_t
    #这里可以定义一个finalpath的cc函数
    def final_path_de_cc(self, fianl_path):
        zui_hou_de_lu_jing_de_cc=[]
        for node in fianl_path:
            #对于这个点求cc：
            A, B=self.vehicle_constraints(node.x,node.y,node.yaw)
            delta_t=0
            # 对所有障碍物
            for obs in self.ganzhi_tuoyuan:
                angle = abs(obs[4])
                angle = angle if angle <= math.pi / 2.0 else math.pi - angle

                delta_tj = math.inf
                for a, b in zip(A, B):
                    a = np.array([a])  # 1*3
                    x = np.array([[obs[0]], [obs[1]], [0.0]])  # 3*1

                    # abs_mat = np.diag([obs[3] * math.sin(angle) + obs[2] * math.cos(angle),
                    #                    obs[2] * math.sin(angle) + obs[3] * math.cos(angle), obs[4]])
                    abs_mat = np.diag([obs[6] * math.sin(angle) + obs[5] * math.cos(angle),
                                       obs[5] * math.sin(angle) + obs[6] * math.cos(angle), obs[4]])
                    sigma = node.conv + abs_mat
                    # sigma = current.conv + np.diag([obs[2], obs[3], obs[4]]) # 3*3
                    erf_item = (a.dot(x).item() - b) / np.sqrt(2 * a.dot(sigma).dot(a.transpose()).item())
                    cc = 0.5 * (1 - erf(erf_item))
                    if cc < delta_tj:
                        delta_tj = cc
                delta_t += delta_tj
            zui_hou_de_lu_jing_de_cc.append(delta_t)
        return zui_hou_de_lu_jing_de_cc

    def get_chance_constrain_from_pu(self, node):
        return node.cc

    def check_chance_constrain(self, current, p_safe):
        return self.get_chance_constrain(current) < 1 - p_safe

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    # 这个函数和rrt也有区别
    def get_heuristic_dis(self, from_node, to_node):
        """
        heuristic distance function
        """
        dis, angle = self.calc_distance_and_angle(from_node, to_node)
        angle = abs(self.angle_wrap(angle - from_node.yaw))
        # heu fun
        t = random.random()
        if len(self.path_end) < self.n_path_when_change_strategy:
            if t < self.k_dis_explore:
                return dis / self.expect_speed + angle / self.expect_turn_rate
            else:
                return from_node.cost
        else:
            if t < self.k_dis_exploit:
                return dis / self.expect_speed + angle / self.expect_turn_rate
            else:
                return from_node.cost

    def get_expect_time_to_goal(self, from_node):
        dis, angle = self.calc_distance_and_angle(from_node, self.end)
        angle = abs(self.angle_wrap(angle - from_node.yaw))
        return (1 - self.k_dis_when_no_path) * from_node.cost + self.k_dis_when_no_path * (
                    dis / self.expect_speed + angle / self.expect_turn_rate)

    def get_cost_lb(self, node):
        """
        cost lower bound is the expected driving time from current node to the goal
        """
        dis, angle = self.calc_distance_and_angle(node, self.end)
        angle = abs(self.angle_wrap(angle - node.yaw))
        return dis / self.expect_speed + angle / self.expect_turn_rate

    def backpropogation(self, node):
        """
        backpropogation to update cost-upper-bound of a path from start to goal
        the first node is the closest to the goal
        """
        min_child_upper_bound = math.inf  # record lowest cost-upper-bound from a node to its childs

        while node is not None and self.calc_distance(node,
                                                      self.end) < self.dis_threshold:  # for nodes in the goal region
            node.cost_ub = node.cost_lb
            min_child_upper_bound = min(min_child_upper_bound + self.delta_time, node.cost_ub)
            node = node.parent
        while node is not None:  # for nodes out of the goal region
            node.cost_ub = min(min_child_upper_bound + self.delta_time + node.cc * self.k_cc, node.cost_ub)
            min_child_upper_bound = min(min_child_upper_bound + self.delta_time, node.cost_ub)
            node = node.parent

    def get_nearest_node_index(self, node_list, rnd_node, n_nearest=1, n_step=0):
        """
        sort tree node according to heuristic, ascending
        get node from sorted nodes ids, node_ids = [0, n_step, 2*n_step, ...]
        if n_nearest = 1, choose the nearest node (return one node [node_id])
        else return node_ids[:n_nearest]
        """
        # dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
        #          ** 2 for node in node_list]
        dlist = [self.get_heuristic_dis(node, rnd_node) for node in node_list]
        if n_nearest > 1:
            sorted_ids = np.argsort(dlist)
            if not len(sorted_ids) > n_nearest:
                return sorted_ids
            else:
                return sorted_ids[range(0, len(sorted_ids), n_step)][:n_nearest]
        else:
            minind = dlist.index(min(dlist))
            return [minind]

    def get_close_to_goal_index(self, node_list):
        """
        get the node close to goal
        """
        dlist = [self.get_expect_time_to_goal(node) for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    def get_random_node(self):
        """
        node sampling
        """

        while True:
            rnd = self.Node(random.uniform(self.min_rand_x, self.max_rand_x),
                            random.uniform(self.min_rand_y, self.max_rand_y),
                            0.0)
            valid = True
            # discard point in obstacle range
            for obs in self.ganzhi_kuang:
                # a = ((rnd.x - obs[0]) * math.cos(obs[4]) + (rnd.y - obs[1]) * math.sin(obs[4]))**2 / obs[2]**2
                # b = ((obs[0] - rnd.x) * math.sin(obs[4]) + (rnd.y - obs[1]) * math.cos(obs[4]))**2 / obs[3]**2
                # if a + b <= 1:
                #     valid = False
                #     break
                if self.is_node_in_vehicle(rnd, obs):
                    valid = False
                    break
            if valid:
                break
        return rnd

    def get_cost(self, time, chance_constraint):
        return time + chance_constraint * self.k_cc

    def angle_check(self, node1, node2, max_angle):
        return np.abs(self.angle_wrap(node1.yaw - node2.yaw)) <= max_angle

    def isRayIntersectsSegment(self, node, s_poi, e_poi):
        poi = [node.x, node.y]
        # 输入：判断点，边起点，边终点，都是[lng,lat]格式数组
        if s_poi[1] == e_poi[1]:  # 排除与射线平行、重合，线段首尾端点重合的情况
            return False
        if s_poi[1] > poi[1] and e_poi[1] > poi[1]:  # 线段在射线上边
            return False
        if s_poi[1] < poi[1] and e_poi[1] < poi[1]:  # 线段在射线下边
            return False
        if s_poi[1] == poi[1] and e_poi[1] > poi[1]:  # 交点为下端点，对应spoint
            return False
        if e_poi[1] == poi[1] and s_poi[1] > poi[1]:  # 交点为下端点，对应epoint
            return False
        if s_poi[0] < poi[0] and e_poi[1] < poi[1]:  # 线段在射线左边
            return False

        xseg = e_poi[0] - (e_poi[0] - s_poi[0]) * (e_poi[1] - poi[1]) / (e_poi[1] - s_poi[1])  # 求交
        if xseg < poi[0]:  # 交点在射线起点的左侧
            return False
        return True  # 排除上述情况之后

    def is_node_in_vehicle(self, node, vehicle):   #第一个参数是节点，第二个参数是障碍物列表
        """
        判断点是否在 vehicle 的 bbox(2D 矩形) 范围内
        """
        w = vehicle[3]
        l = vehicle[2]
        x = vehicle[0]
        y = vehicle[1]
        yaw = vehicle[4]
        p = []
        # 此处点顺序为顺时针
        p.append([
            x + l * math.cos(yaw) + w * math.sin(yaw),
            y + l * math.sin(yaw) - w * math.cos(yaw)
        ])
        p.append([
            x - l * math.cos(yaw) + w * math.sin(yaw),
            y - l * math.sin(yaw) - w * math.cos(yaw)
        ])
        p.append([
            x - l * math.cos(yaw) - w * math.sin(yaw),
            y - l * math.sin(yaw) + w * math.cos(yaw)
        ])
        p.append([
            x + l * math.cos(yaw) - w * math.sin(yaw),
            y + l * math.sin(yaw) + w * math.cos(yaw)
        ])
        p.append([
            x + l * math.cos(yaw) + w * math.sin(yaw),
            y + l * math.sin(yaw) - w * math.cos(yaw)
        ])

        intersection = 0
        for i in range(len(p) - 1):
            s_poi = p[i]
            e_poi = p[i + 1]
            if self.isRayIntersectsSegment(node, s_poi, e_poi):
                intersection += 1
        return True if intersection % 2 == 1 else False

    def in_place(self, node):
        """
        check if a node in planning place
        """
        valid = node.x < self.max_rand_x and node.x > self.min_rand_x and \
                node.y < self.max_rand_y and node.y > self.min_rand_y
        valid_2 = True
        # check collision to vehicle bbox
        for obs in self.ganzhi_kuang:
            # # ellipse
            # a = ((node.x - obs[0]) * math.cos(obs[4]) + (node.y - obs[1]) * math.sin(obs[4]))**2 / obs[2]**2
            # b = ((obs[0] - node.x) * math.sin(obs[4]) + (node.y - obs[1]) * math.cos(obs[4]))**2 / obs[3]**2
            # if a + b <= 1.0:
            #     valid_2 = False
            #     break
            # bbox
            if self.is_node_in_vehicle(node, obs):
                valid_2 = False
                break
        return valid and valid_2

    def egocar_to_obs(self,ego_car_node):
        return [ego_car_node.x,ego_car_node.y,4.51/2,1.0,ego_car_node.yaw]

    #这里添加一个碰撞检测的函数，我的意图是检测障碍物是不是在车里
    # !!! 因为要检测和感知的碰撞检测  和  真值的碰撞检测  所以障碍物的列表页应该是传递进去的！！！
    def peng_zhuang_jian_ce_oie(self,ego_car_node, list_for_check):  #obs是障碍物列表，第二个参数也是节点，在函数内部我会将它转化为障碍物那样的列表

        ## 在给定节点的情况下，计算四个角点
        #w = self.car.w / 2.0
        is_pengzhuang = 0  # sum(min delte_tj)
        for obs in list_for_check:
            p0 = [
                obs[0] + obs[2] * math.cos(obs[4]) + obs[3] * math.sin(obs[4]),
                obs[1] + obs[2] * math.sin(obs[4]) - obs[3] * math.cos(obs[4])
            ]
            p1 = [
                obs[0] + obs[2] * math.cos(obs[4]) - obs[3] * math.sin(obs[4]),
                obs[1] + obs[2] * math.sin(obs[4]) + obs[3] * math.cos(obs[4])
            ]
            p2 = [
                obs[0] - obs[2] * math.cos(obs[4]) - obs[3] * math.sin(obs[4]),
                obs[1] - obs[2] * math.sin(obs[4]) + obs[3] * math.cos(obs[4])
            ]
            p3 = [
                obs[0] - obs[2] * math.cos(obs[4]) + obs[3] * math.sin(obs[4]),
                obs[1] - obs[2] * math.sin(obs[4]) - obs[3] * math.cos(obs[4])
            ]
            # 得到了障碍物四个角点的坐标
            jiao1 = self.Node(p0[0], p0[1], 0.0)
            jiao2 = self.Node(p1[0], p1[1], 0.0)
            jiao3 = self.Node(p2[0], p2[1], 0.0)
            jiao4 = self.Node(p3[0], p3[1], 0.0)
            # 得到了四个角节点
            #得到障碍物四个角点节点的情况下，检测四个角点是不是在障碍物中

            # cal for each obs
            ego_car_obs=self.egocar_to_obs(ego_car_node)
            if self.is_node_in_vehicle(jiao1, ego_car_obs):    #这个函数第一个参数是节点，但是第二个参数应该是一个列表【障碍物的x，障碍物的y，障碍物矩形的长半，短半，yaw 】
                is_pengzhuang += 1
            if self.is_node_in_vehicle(jiao2, ego_car_obs):
                is_pengzhuang += 1
            if self.is_node_in_vehicle(jiao3, ego_car_obs):
                is_pengzhuang += 1
            if self.is_node_in_vehicle(jiao4, ego_car_obs):
                is_pengzhuang += 1
        return is_pengzhuang

    #碰撞检测，车是否在障碍物中
    def peng_zhuang_jian_ce_eio(self, current,list_for_lisk):   #这里的current就是ego_car_node
        w = self.car.w / 2.0
        p0 = [
            current.x + self.car.l_f * math.cos(current.yaw) + w * math.sin(current.yaw),
            current.y + self.car.l_f * math.sin(current.yaw) - w * math.cos(current.yaw)
        ]
        p1 = [
            current.x + self.car.l_f * math.cos(current.yaw) - w * math.sin(current.yaw),
            current.y + self.car.l_f * math.sin(current.yaw) + w * math.cos(current.yaw)
        ]
        p2 = [
            current.x - self.car.l_r * math.cos(current.yaw) - w * math.sin(current.yaw),
            current.y - self.car.l_r * math.sin(current.yaw) + w * math.cos(current.yaw)
        ]
        p3 = [
            current.x - self.car.l_r * math.cos(current.yaw) + w * math.sin(current.yaw),
            current.y - self.car.l_r * math.sin(current.yaw) - w * math.cos(current.yaw)
        ]
        # 得到了四个角点的坐标
        jiao1 = self.Node(p0[0],p0[1],0.0)
        jiao2 = self.Node(p1[0], p1[1], 0.0)
        jiao3 = self.Node(p2[0], p2[1], 0.0)
        jiao4 = self.Node(p3[0], p3[1], 0.0)
        #得到了四个角节点
       # A, B = self.vehicle_constraints(current.x, current.y, current.yaw)
        delta_t = 0  # sum(min delte_tj)
        # cal for each obs
        for obs in list_for_lisk:
            if self.is_node_in_vehicle(jiao1,obs):
                delta_t += 1
            if self.is_node_in_vehicle(jiao2, obs):
                delta_t += 1
            if self.is_node_in_vehicle(jiao3, obs):
                delta_t += 1
            if self.is_node_in_vehicle(jiao4, obs):
                delta_t += 1
        return delta_t

    #碰撞检测，将车是否在障碍物中和障碍物是否在车中进行总和
    def peng_zhuang_jian_ce(self, ego_car_node,list_for_check):
        if self.peng_zhuang_jian_ce_oie(ego_car_node,list_for_check)>=1:      #障碍物在车里
            return True
        if self.peng_zhuang_jian_ce_eio(ego_car_node,list_for_check)>=1:      #车在障碍物里
            return True
        return False

    def safe_steer(self, node):
        if node.parent:
            _, jiaodu = self.calc_distance_and_angle(node.parent, node)
            xs = np.ones(10) * node.x if node.parent.x == node.x else \
                np.arange(node.parent.x, node.x, (node.x - node.parent.x) / 10.0)
            ys = np.ones(10) * node.y if node.parent.y == node.y else \
                np.arange(node.parent.y, node.y, (node.y - node.parent.y) / 10.0)
            for x, y in zip(xs, ys):

                t_node = self.Node(x, y, jiaodu)  #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!刚开始一直找不到路径，是因为我把角度默认为了0，显然不应该默认为0.
                """
                for obs in self.obstacle_list:    #！！！！！！！这里其实还缺少一个（障碍物的角点节点是否在自车中）的判断检测
                    if self.is_node_in_vehicle(t_node, obs):
                        return False
                for obs in self.obstacle_list:
                    if self.peng_zhuang_jian_ce_oie(obs,t_node) >= 1:
                        return False
                if self.get_chance_constrain(t_node) >= 1:      # =----------------------------------------这里我将修改的东西又注释了，在一个中间节点和from节点之间的十个点是不是也要满足不能和障碍物相撞的前提呢？注释的原因：扩展困难！
                    return False
                """
                if self.peng_zhuang_jian_ce(t_node,self.ganzhi_kuang):
                    return False
        return True


    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        """
        arg1:from_node -> arg2:to_node
        """
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    @staticmethod
    def calc_distance(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        return math.hypot(dx, dy)

    @staticmethod
    def angle_wrap(angle):
        while angle <= -math.pi:
            angle = angle + 2 * math.pi
        while angle > math.pi:
            angle = angle - 2 * math.pi
        return angle


