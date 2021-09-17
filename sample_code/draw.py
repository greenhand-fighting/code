import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.legend_handler import HandlerPatch
import math
from matplotlib.patches import Ellipse, Rectangle
import numpy as np
class AnyObject:
    pass
class AnyObject1:
    pass
class AnyObject2:
    pass
class AnyObject3:
    pass
class AnyObjectHandler:
    def legend_artist(self, legend, orig_handle, fontsize, handlebox):
        x0, y0 = handlebox.xdescent, handlebox.ydescent
        width, height = handlebox.width, handlebox.height
        patch = mpatches.Rectangle([x0, y0], width, height, facecolor='white',
                                   edgecolor='black', lw=1,
                                   transform=handlebox.get_transform())
        handlebox.add_artist(patch)
        return patch
class AnyObjectHandler1:
    def legend_artist(self, legend, orig_handle, fontsize, handlebox):
        x0, y0 = handlebox.xdescent, handlebox.ydescent
        width, height = handlebox.width, handlebox.height
        patch = mpatches.Rectangle([x0, y0], width, height, facecolor='white',
                                   edgecolor='orange', lw=1,
                                   transform=handlebox.get_transform())
        handlebox.add_artist(patch)
        return patch
# 预测
class AnyObjectHandler2:
    def legend_artist(self, legend, orig_handle, fontsize, handlebox):
        x0, y0 = handlebox.xdescent, handlebox.ydescent
        width, height = handlebox.width, handlebox.height
        patch = mpatches.Rectangle([x0, y0], width, height, facecolor='white',
                                   edgecolor='red', lw=1,
                                   transform=handlebox.get_transform())
        handlebox.add_artist(patch)
        return patch
# green 真值
class AnyObjectHandler3:
    def legend_artist(self, legend, orig_handle, fontsize, handlebox):
        x0, y0 = handlebox.xdescent, handlebox.ydescent
        width, height = handlebox.width, handlebox.height
        patch = mpatches.Rectangle([x0, y0], width, height, facecolor='white',
                                   edgecolor='green', lw=1,
                                   transform=handlebox.get_transform())
        handlebox.add_artist(patch)
        return patch
class HandlerEllipse(HandlerPatch):
    def create_artists(self, legend, orig_handle,
                       xdescent, ydescent, width, height, fontsize, trans):
        center = 0.5 * width - 0.5 * xdescent, 0.5 * height - 0.5 * ydescent
        p = mpatches.Ellipse(xy=center, width=width + xdescent,
                             height=height + ydescent)
        self.update_prop(p, orig_handle, legend)
        p.set_transform(trans)
        return [p]


class Artist:
    def __init__(self,rrt_object, ganzhi_kuang,zhenzhi_kuang, tuoyuan,draw_area = [-25, 25, -25, 25], if_draw_tree=1, is_clrrt=1):

        self.draw_area=draw_area
        self.rrt_object=rrt_object
        self.ganzhi_kuang=ganzhi_kuang
        self.zhenzhi_kuang=zhenzhi_kuang
        self.draw_tree=if_draw_tree
        self.is_clrrt=is_clrrt
        self.tuoyuan=tuoyuan
        pass

    def plot_arrow(self, x, y, yaw, length=0.5, width=0.25, fc="r", ec="k"):
        """
        Plot arrow
        """
        if not isinstance(x, float):
            for (ix, iy, iyaw) in zip(x, y, yaw):
                self.plot_arrow(ix, iy, iyaw)
        else:
            plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                      fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)

    def draw_graph(self, is_cross, is_first_graph, rnd=None):
        plt.clf()
        if is_cross:
            ax=plt.axes()
        else :
            ax = plt.axes([0.3, 0.1, 8 / 50, 8 / 10])

        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if self.is_clrrt==0:
            for obs in self.tuoyuan:
                ellipse = Ellipse((obs[0], obs[1]), obs[2] * 2.0, obs[3] * 2.0, math.degrees(obs[4]),color='#1f77b4')
                ax.add_patch(ellipse)

        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        if self.draw_tree and is_first_graph:
            for node in self.rrt_object.node_list:
                plt.plot(node.x, node.y, "*b")
                self.plot_arrow(node.x, node.y, node.yaw, fc='b')
      #  plt.title("With considering uncertainty (represented by ellipse)\npurrt    P_safe:%f" % self.rrt_object.p_safe)
        plt.plot(self.rrt_object.start.x, self.rrt_object.start.y, "xr")
        self.plot_arrow(self.rrt_object.start.x, self.rrt_object.start.y, self.rrt_object.start.yaw, fc='r')
        plt.plot(self.rrt_object.end.x, self.rrt_object.end.y, "xg")
        self.plot_arrow(self.rrt_object.end.x, self.rrt_object.end.y, self.rrt_object.end.yaw, fc='g')
        plt.axis("equal")
        plt.axis([self.rrt_object.min_rand_x, self.rrt_object.max_rand_x, self.rrt_object.min_rand_y, self.rrt_object.max_rand_y])
     #   plt.grid(True)
        plt.pause(0.01)

    def draw_path(self):
        for node in self.rrt_object.path:
            plt.plot(node.x, node.y, "*r")
            self.plot_arrow(node.x, node.y, node.yaw, fc='r')
     #   plt.title("With considering uncertainty (represented by ellipse)\npurrt    P_safe:%f" % self.rrt_object.p_safe)
        plt.plot(self.rrt_object.start.x, self.rrt_object.start.y, "*y", markersize=10, label='start')
        self.plot_arrow(self.rrt_object.start.x, self.rrt_object.start.y, self.rrt_object.start.yaw, fc='y')
        plt.plot(self.rrt_object.end.x, self.rrt_object.end.y, "*g", markersize=10, label='goal')
        self.plot_arrow(self.rrt_object.end.x, self.rrt_object.end.y, self.rrt_object.end.yaw, fc='g')
        plt.legend(loc='upper right')

    def draw_vehicle(self,obs_list):
        for obs in obs_list:
            w = obs[3]
            l = obs[2]
            x = obs[0]
            y = obs[1]
            yaw = obs[4]
            p0 = [
                x + l * math.cos(yaw) + w * math.sin(yaw),
                y + l * math.sin(yaw) - w * math.cos(yaw)
            ]
            p1 = [
                x + l * math.cos(yaw) - w * math.sin(yaw),
                y + l * math.sin(yaw) + w * math.cos(yaw)
            ]
            p2 = [
                x - l * math.cos(yaw) - w * math.sin(yaw),
                y - l * math.sin(yaw) + w * math.cos(yaw)
            ]
            p3 = [
                x - l * math.cos(yaw) + w * math.sin(yaw),
                y - l * math.sin(yaw) - w * math.cos(yaw)
            ]
            plt.plot([p0[0], p1[0]], [p0[1], p1[1]], 'k', color="r")
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k', color="r")
            plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k', color="r")
            plt.plot([p3[0], p0[0]], [p3[1], p0[1]], 'k', color="r")

    def draw_ground_true(self, obs_list):
        x1 = True
        for obs in obs_list:
            # x=0
            w = obs[3]
            l = obs[2]
            x = obs[0]
            y = obs[1]
            yaw = obs[4]
            p0 = [
                x + l * math.cos(yaw) + w * math.sin(yaw),
                y + l * math.sin(yaw) - w * math.cos(yaw)
            ]
            p1 = [
                x + l * math.cos(yaw) - w * math.sin(yaw),
                y + l * math.sin(yaw) + w * math.cos(yaw)
            ]
            p2 = [
                x - l * math.cos(yaw) - w * math.sin(yaw),
                y - l * math.sin(yaw) + w * math.cos(yaw)
            ]
            p3 = [
                x - l * math.cos(yaw) + w * math.sin(yaw),
                y - l * math.sin(yaw) - w * math.cos(yaw)
            ]
            plt.plot([p0[0], p1[0]], [p0[1], p1[1]], 'k', color="green", lw='1')
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k', color="green", lw='1')
            plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k', color="green", lw='1')
            plt.plot([p3[0], p0[0]], [p3[1], p0[1]], 'k', color="green", lw='1')

    def draw_carsize_of_final_path(self, obs_list):
        for obs in obs_list:
            # x=0
            w = 2 / 2.0
            l = 4.51 / 2.0
            x = obs.x
            y = obs.y
            yaw = obs.yaw
            p0 = [
                x + l * math.cos(yaw) + w * math.sin(yaw),
                y + l * math.sin(yaw) - w * math.cos(yaw)
            ]
            p1 = [
                x + l * math.cos(yaw) - w * math.sin(yaw),
                y + l * math.sin(yaw) + w * math.cos(yaw)
            ]
            p2 = [
                x - l * math.cos(yaw) - w * math.sin(yaw),
                y - l * math.sin(yaw) + w * math.cos(yaw)
            ]
            p3 = [
                x - l * math.cos(yaw) + w * math.sin(yaw),
                y - l * math.sin(yaw) - w * math.cos(yaw)
            ]
            plt.plot([p0[0], p1[0]], [p0[1], p1[1]], 'k', color="black", lw='1.2')
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k', color="black", lw='1.2')
            plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k', color="black", lw='1.2')
            plt.plot([p3[0], p0[0]], [p3[1], p0[1]], 'k', color="black", lw='1.2')

    def play_cross(self):
        is_cross=True
        plt.figure(1, figsize=(6, 6))
        self.draw_graph(is_cross,is_first_graph=True)
        self.draw_path()
        self.draw_vehicle(self.ganzhi_kuang)
        self.draw_ground_true(self.zhenzhi_kuang)
        self.draw_carsize_of_final_path(self.rrt_object.path)
        plt.axis("equal")
        plt.axis([self.draw_area[0], self.draw_area[1], self.draw_area[2], self.draw_area[3]])
        # 画路
        plt.plot([7, 7], [12, 25], color="grey")
        plt.plot([12, 25], [7, 7], color="grey")
        plt.plot([-7, -7], [12, 25], color="grey")
        plt.plot([-12, -25], [7, 7], color="grey")
        plt.plot([-7, -7], [-12, -25], color="grey")
        plt.plot([-12, -25], [-7, -7], color="grey")
        plt.plot([7, 7], [-12, -25], color="grey")
        plt.plot([12, 25], [-7, -7], color="grey")
        plt.plot([7, 12], [-12, -7], color="grey")
        plt.plot([-7, -12], [-12, -7], color="grey")
        plt.plot([-7, -12], [12, 7], color="grey")
        plt.plot([7, 12], [12, 7], color="grey")
        plt.plot([-3.5, -3.5], [7, 25], "--", color="grey")
        plt.plot([0, 0], [7, 25], color="grey")
        plt.plot([3.5, 3.5], [7, 25], "--", color="grey")
        plt.plot([7, 25], [3.5, 3.5], "--", color="grey")
        plt.plot([7, 25], [0, 0], color="grey")
        plt.plot([7, 25], [-3.5, -3.5], "--", color="grey")
        plt.plot([3.5, 3.5], [-7, -25], "--", color="grey")
        plt.plot([0, 0], [-7, -25], color="grey")
        plt.plot([-3.5, -3.5], [-7, -25], "--", color="grey")
        plt.plot([-7, -25], [3.5, 3.5], "--", color="grey")
        plt.plot([-7, -25], [0, 0], color="grey")
        plt.plot([-7, -25], [-3.5, -3.5], "--", color="grey")
        # 画规划空间范围
        plt.plot([-7, -7], [16, -1.75], "--", color="orange")
        plt.plot([-7, -1.75], [-1.75, -7], "--", color="orange")
        plt.plot([-1.75, 23], [-7, -7], "--", color="orange")
        plt.plot([23, 23], [-7, 0], "--", color="orange")
        plt.plot([23, 7], [0, 0], "--", color="orange")
        plt.plot([7, 0], [0, 7], "--", color="orange")
        plt.plot([0, 0], [7, 16], "--", color="orange")
        plt.plot([0, -7], [16, 16], "--", color="orange")
        # area = [-10, 10, -10, 20]
        # 画图例的-------------------------
        c = mpatches.Circle((0.5, 0.5), 0.25, facecolor="#1f77b4",
                            edgecolor="#1f77b4", linewidth=3)
        # plt.gca().add_patch(c)

        # 第一个图例
        if self.is_clrrt:
            first_legend = plt.legend([AnyObject(), AnyObject2(), AnyObject3()],
                                      ['ego_car(trajectory)', "prediction", "ground truth"],
                                      handler_map={AnyObject: AnyObjectHandler()
                                                   #  , AnyObject1: AnyObjectHandler1()
                                          , AnyObject2: AnyObjectHandler2()
                                          , AnyObject3: AnyObjectHandler3()}
                                      , loc='upper right', fontsize=15
                                      # , bbox_to_anchor=(1.01, 1), loc='upper left', borderaxespad=0.
                                      )
            pass
        else:
            first_legend = plt.legend([AnyObject(), AnyObject2(), AnyObject3(), c],
                                      ['ego_car(trajectory)', "prediction", "ground truth",
                                       "spatial uncertainty"],
                                      handler_map={AnyObject: AnyObjectHandler()
                                                   #  , AnyObject1: AnyObjectHandler1()
                                          , AnyObject2: AnyObjectHandler2()
                                          , AnyObject3: AnyObjectHandler3()
                                          , mpatches.Circle: HandlerEllipse()}
                                      , loc='upper right', fontsize=15
                                      # , bbox_to_anchor=(1.01, 1), loc='upper left', borderaxespad=0.
                                      )

        plt.gca().add_artist(first_legend)

        #     line2, = plt.plot([3, 2, 1], label="Line 2", linewidth=4)
        start_star, = plt.plot(-100, 0, "*y", markersize=10, label='start')
        goal_star, = plt.plot(-100, 0, "*g", markersize=10, label='goal')
        mid_star, = plt.plot(-100, 0, "*r", markersize=10, label='path point')
        tree_node_star, = plt.plot(-1000, 0, "*b", markersize=10, label='tree_node')
        # 第二个图例
        if self.draw_tree:
            plt.legend(handles=[
                # line2
                start_star
                , goal_star
                , mid_star
                , tree_node_star
            ]
                , loc='lower left'
                , fontsize=15
                #    , bbox_to_anchor=(1.01, 0), loc='lower left', borderaxespad=0.
            )
        #    plt.show()
            pass
        else:
            plt.legend(handles=[
                # line2
                start_star
                , goal_star
                , mid_star
                #    , tree_node_star
            ]
                , loc='lower left'
                , fontsize=15
                #    , bbox_to_anchor=(1.01, 0), loc='lower left', borderaxespad=0.
            )
        #    plt.show()
        # 画图例完成---------------
        plt.figure(2, figsize=(6, 6))
        self.draw_graph(is_cross,is_first_graph=False)
        self.draw_vehicle(self.ganzhi_kuang)
        self.draw_ground_true(self.zhenzhi_kuang)

        tmp = [node.cc for node in self.rrt_object.path]  # 从这个可以看出这个finalpath里面都是一些节点
        # print(tmp)
        print(len(self.rrt_object.path))
        path_min = np.min(tmp)
        path_max = np.max(tmp)
        path_avg = np.average(tmp)

        # plt.axes([0.3, 0.1, 8 / 50, 8 / 10.55])
        plt.scatter([node.x for node in self.rrt_object.node_list],
                    [node.y for node in self.rrt_object.node_list],
                    s=3,
                    c=[node.cc for node in self.rrt_object.node_list],
                    cmap='jet')
        plt.plot([node.x for node in self.rrt_object.path],
                 [node.y for node in self.rrt_object.path],
                 c='k',
                 label="path risk value:\nmin: %.6f\nmax: %.6f\navg: %.6f" % (path_min, path_max, path_avg))
        plt.colorbar()
        plt.axis("equal")
        plt.axis([self.draw_area[0], self.draw_area[1], self.draw_area[2], self.draw_area[3]])
        plt.legend(loc='upper right', fontsize=15)
       # plt.grid(True)
        plt.plot([7, 7], [12, 25], color="grey")
        plt.plot([12, 25], [7, 7], color="grey")
        plt.plot([-7, -7], [12, 25], color="grey")
        plt.plot([-12, -25], [7, 7], color="grey")
        plt.plot([-7, -7], [-12, -25], color="grey")
        plt.plot([-12, -25], [-7, -7], color="grey")
        plt.plot([7, 7], [-12, -25], color="grey")
        plt.plot([12, 25], [-7, -7], color="grey")
        plt.plot([7, 12], [-12, -7], color="grey")
        plt.plot([-7, -12], [-12, -7], color="grey")
        plt.plot([-7, -12], [12, 7], color="grey")
        plt.plot([7, 12], [12, 7], color="grey")
        plt.plot([-3.5, -3.5], [7, 25], "--", color="grey")
        plt.plot([0, 0], [7, 25], color="grey")
        plt.plot([3.5, 3.5], [7, 25], "--", color="grey")
        plt.plot([7, 25], [3.5, 3.5], "--", color="grey")
        plt.plot([7, 25], [0, 0], color="grey")
        plt.plot([7, 25], [-3.5, -3.5], "--", color="grey")
        plt.plot([3.5, 3.5], [-7, -25], "--", color="grey")
        plt.plot([0, 0], [-7, -25], color="grey")
        plt.plot([-3.5, -3.5], [-7, -25], "--", color="grey")
        plt.plot([-7, -25], [3.5, 3.5], "--", color="grey")
        plt.plot([-7, -25], [0, 0], color="grey")
        plt.plot([-7, -25], [-3.5, -3.5], "--", color="grey")
        # 画规划空间范围
        plt.plot([-7, -7], [16, -1.75], "--", color="orange")
        plt.plot([-7, -1.75], [-1.75, -7], "--", color="orange")
        plt.plot([-1.75, 23], [-7, -7], "--", color="orange")
        plt.plot([23, 23], [-7, 0], "--", color="orange")
        plt.plot([23, 7], [0, 0], "--", color="orange")
        plt.plot([7, 0], [0, 7], "--", color="orange")
        plt.plot([0, 0], [7, 16], "--", color="orange")
        plt.plot([0, -7], [16, 16], "--", color="orange")

        plt.show()

        pass

    def play_long(self):
        is_cross=False
        # 开始绘制第一章图
        self.draw_graph(is_cross,is_first_graph=True)
        self.draw_path()
        self.draw_vehicle(self.ganzhi_kuang)
        self.draw_ground_true(self.zhenzhi_kuang)
        self.draw_carsize_of_final_path(self.rrt_object.path)
        plt.axis([self.draw_area[0], self.draw_area[1], self.draw_area[2], self.draw_area[3]])
        # 画路
        plt.plot([0, 0], [30, 105], color="grey",lw='1.5')
        plt.plot([14.4, 14.4], [30, 105], color="grey",lw='1.5')
        plt.plot([3.6, 3.6], [30, 105], "--", color="grey",lw='1.2')
        plt.plot([7.2, 7.2], [30, 105], "--", color="grey",lw='1.5')
        plt.plot([10.8, 10.8], [30, 105], "--", color="grey",lw='1.5')
        # 画图例的-------------------------
        c = mpatches.Circle((0.5, 0.5), 0.25, facecolor="#1f77b4",
                            edgecolor="#1f77b4", linewidth=3)
        # plt.gca().add_patch(c)

        # 第一个图例
        if self.is_clrrt:
            first_legend = plt.legend([AnyObject(), AnyObject2(), AnyObject3()],
                                      ['ego_car(trajectory)', "prediction", "ground truth"],
                                      handler_map={AnyObject: AnyObjectHandler()
                                                   #  , AnyObject1: AnyObjectHandler1()
                                          , AnyObject2: AnyObjectHandler2()
                                          , AnyObject3: AnyObjectHandler3()}
                                      , loc='upper right', fontsize=15
                                      # , bbox_to_anchor=(1.01, 1), loc='upper left', borderaxespad=0.
                                      )
            pass
        else:
            first_legend = plt.legend([AnyObject(), AnyObject2(), AnyObject3(), c],
                                      ['ego_car(trajectory)', "prediction", "ground truth",
                                       "spatial uncertainty"],
                                      handler_map={AnyObject: AnyObjectHandler()
                                                   #  , AnyObject1: AnyObjectHandler1()
                                          , AnyObject2: AnyObjectHandler2()
                                          , AnyObject3: AnyObjectHandler3()
                                          , mpatches.Circle: HandlerEllipse()}
                                      , loc='upper right', fontsize=12
                                      # , bbox_to_anchor=(1.01, 1), loc='upper left', borderaxespad=0.
                                      )

        plt.gca().add_artist(first_legend)

        #     line2, = plt.plot([3, 2, 1], label="Line 2", linewidth=4)
        start_star, = plt.plot(-100, 0, "*y", markersize=10, label='start')
        goal_star, = plt.plot(-100, 0, "*g", markersize=10, label='goal')
        mid_star, = plt.plot(-100, 0, "*r", markersize=10, label='path point')
        tree_node_star, = plt.plot(-1000, 0, "*b", markersize=10, label='tree_node')
        # 第二个图例
        if self.draw_tree:
            plt.legend(handles=[
                # line2
                start_star
                , goal_star
                , mid_star
                , tree_node_star
            ]
                , loc='lower left'
                , fontsize=12
                #    , bbox_to_anchor=(1.01, 0), loc='lower left', borderaxespad=0.
            )
            plt.show()
            pass
        else:
            plt.legend(handles=[
                # line2
                start_star
                , goal_star
                , mid_star
                #    , tree_node_star
            ]
                , loc='lower left'
                , fontsize=15
                #    , bbox_to_anchor=(1.01, 0), loc='lower left', borderaxespad=0.
            )
            plt.show()
        # 画图例完成---------------
        # 第一张图绘制完成

        plt.figure(2, figsize=(6, 6))
        self.draw_graph(is_cross,is_first_graph=False)
        self.draw_vehicle(self.ganzhi_kuang)
        self.draw_ground_true(self.zhenzhi_kuang)

        tmp = [node.cc for node in self.rrt_object.path]  # 从这个可以看出这个finalpath里面都是一些节点
        # print(tmp)
        print(len(self.rrt_object.path))
        path_min = np.min(tmp)
        path_max = np.max(tmp)
        path_avg = np.average(tmp)

        # plt.axes([0.3, 0.1, 8 / 50, 8 / 10.55])
        plt.scatter([node.x for node in self.rrt_object.node_list],
                    [node.y for node in self.rrt_object.node_list],
                    s=3,
                    c=[node.cc for node in self.rrt_object.node_list],
                    cmap='jet')
        plt.plot([node.x for node in self.rrt_object.path],
                 [node.y for node in self.rrt_object.path],
                 c='k',
                 label="path risk value:\nmin: %.6f\nmax: %.6f\navg: %.6f" % (path_min, path_max, path_avg))
        plt.colorbar()
        plt.axis("equal")
        # plt.axis([area[0], area[1], area[2], area[3]])
        plt.axis([self.draw_area[0], self.draw_area[1], self.draw_area[2], self.draw_area[3]])
        # 画路
        plt.plot([0, 0], [30, 105], color="grey", lw='1.5')
        plt.plot([14.4, 14.4], [30, 105], color="grey", lw='1.5')
        plt.plot([3.6, 3.6], [30, 105], "--", color="grey", lw='1.2')
        plt.plot([7.2, 7.2], [30, 105], "--", color="grey", lw='1.5')
        plt.plot([10.8, 10.8], [30, 105], "--", color="grey", lw='1.5')
        plt.legend(loc='upper right')
        # plt.grid(True)
        plt.show()
        # 第二张图绘制完成


if __name__=="__main__":

    x=Artist()
