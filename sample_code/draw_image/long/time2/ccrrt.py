import sys
sys.path.append("../../..")
print(sys.path)
import input, draw, calulate
import CCRRT, CLRRT, PURRT
import numpy as np
car=CCRRT.Vehicle(input.ego_car_size[0],input.ego_car_size[1],input.ego_car_size[2])

# 值的传递：
s=input.starting_point_time2
g=input.goal_point_time2
r=input.random_sample_area2
rt=input.rengong_tuoyuan2
gk=input.ganzhi_kuang2
gt=input.ganzhi_tuoyuan2
zk=input.zhenzhi_kuang
# 值传递完毕

print(r)
tree=CCRRT.CcrrtGo(car=car, start=s,goal=g, ganzhi_kuang=gk, ganzhi_tuoyuan=gt, rand_area=r,rengong_tuoyuan=rt)
tree.max_n_node=5000
tree.p_safe=0.96
tree.planning(animation=False)
print(len(tree.path))
x=draw.Artist(tree,gk, zk, tuoyuan=rt, if_draw_tree=0, is_clrrt=0,draw_area=[0, 15, 30, 100])  # 坐标轴 draw_area = [-25, 25, -25, 25]  # x-min x-max y-min y-max
x.play_long()