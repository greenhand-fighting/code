import matplotlib.pyplot as plt
import numpy as np
import math

def draw_ground_true(obs_list):
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

"""
直道的！
ax = plt.axes([0.3, 0.1, 8 / 50, 8 / 10])
start_star,=plt.plot(12.25, 35, "*b",markersize=15,label="time1_start")
start_star2,=plt.plot(5.5, 50.7, "*r",markersize=15,label="time2_start")
start_star3,=plt.plot(1.86, 61.3, "*g",markersize=15,label="time3_start")
start_star4,=plt.plot(9.066, 67.66, "*y",markersize=15,label="time4_start")

e1,=plt.plot(9.0, 70.0, "ob",markersize=10,label="time1_end")
e2,=plt.plot(5.0, 85.0, "or",markersize=10,label="time2_end")
e3,=plt.plot(12.0, 95.0, "og",markersize=10,label="time3_end")
e4,=plt.plot(2.0, 100.0, "oy",markersize=10,label="time4_end")
# 画路
plt.plot([0, 0], [30, 105], color="grey",lw='1.5')
plt.plot([14.4, 14.4], [30, 105], color="grey",lw='1.5')
plt.plot([3.6, 3.6], [30, 105], "--", color="grey",lw='1.2')
plt.plot([7.2, 7.2], [30, 105], "--", color="grey",lw='1.5')
plt.plot([10.8, 10.8], [30, 105], "--", color="grey",lw='1.5')


# 真值框
zhenzhi_kuang = [
            (1.9, 75.0, 4.72 / 2, 1.89 / 2, np.deg2rad(90.0)),
            (5.5, 45.0, 4.19 / 2, 1.82 / 2, np.deg2rad(90.0)),
            (5.5, 65.0, 4.79 / 2, 2.16 / 2, np.deg2rad(90.0)),
            (5.5, 95.0, 5.36 / 2, 2.03 / 2, np.deg2rad(90.0)),
            (9.1, 55.0, 4.86 / 2, 2.03 / 2, np.deg2rad(90.0)),
            (9.1, 85.0, 3.99 / 2, 1.85 / 2, np.deg2rad(90.0)),
            (12.7, 45.0, 4.18 / 2, 1.99 / 2, np.deg2rad(90.0)),
            (12.7, 65.0, 4.61 / 2, 2.24 / 2, np.deg2rad(90.0)),
        ]   # [x,y,length/2,width/2,yaw]
draw_ground_true(zhenzhi_kuang)

first_legend = plt.legend(handles=[start_star,start_star2,start_star3,start_star4],
                                      loc='lower left'
                                      #, fontsize=12
                                      # , bbox_to_anchor=(1.01, 1), loc='upper left', borderaxespad=0.
                                      )
plt.gca().add_artist(first_legend)
plt.legend(handles=[e1,e2,e3,e4 ]
                , loc='upper right'
                #, fontsize=12
                #    , bbox_to_anchor=(1.01, 0), loc='lower left', borderaxespad=0.
            )
# plt.legend()
plt.axis("equal")
plt.axis([0,15,30,105])
plt.show()

"""

# 十字路口的！
ax = plt.axes()
start_star,=plt.plot(-1.75, 13.0, "*b",markersize=15,label="time1_start")
start_star2,=plt.plot(0.25, -2.0, "*r",markersize=15,label="time2_start")

e1,=plt.plot(4.5, -4, "ob",markersize=10,label="time1_end")
e2,=plt.plot(20.0, -1.75, "or",markersize=10,label="time2_end")

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


# 真值框
zhenzhi_kuang_cross = [
            (-5.3, -8.5, 4.8557 / 2, 2.0323 / 2, -np.pi / 2),
            (-5.3, 9.0, 4.7175 / 2, 1.895 / 2, -np.pi / 2),
            (1.8, -8.5, 4.611 / 2, 2.2417 / 2, np.pi / 2),
            (1.4, 1.2, 4.974 / 2, 2.0384 / 2, -0.890120),
            (6.4, -9.3, 3.8058 / 2, 1.9703 / 2, 1.20428),
            (11.0, -1.8, 3.9877 / 2, 1.851 / 2, 0.0),
            (20.0, -5.3, 3.9877 / 2, 1.851 / 2, 0.0),
            # (-1.75, 13.0, 4.5135, 2.0068, -np.pi/2)
        ]  # 真值框，用于检测是不是和真值发生碰撞
draw_ground_true(zhenzhi_kuang_cross)

first_legend = plt.legend(handles=[start_star,start_star2],
                                      loc='lower left'
                                      #, fontsize=12
                                      # , bbox_to_anchor=(1.01, 1), loc='upper left', borderaxespad=0.
                                      )
plt.gca().add_artist(first_legend)
plt.legend(handles=[e1,e2]
                , loc='upper right'
                #, fontsize=12
                #    , bbox_to_anchor=(1.01, 0), loc='lower left', borderaxespad=0.
            )
# plt.legend()
plt.axis("equal")
plt.axis([-25, 25, -25, 25])
plt.show()