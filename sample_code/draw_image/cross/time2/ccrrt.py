import sys
print(sys.path)
sys.path.append("../../..")
import input, draw, calulate
import CCRRT, CLRRT, PURRT
import numpy as np
car=CCRRT.Vehicle(input.ego_car_size[0],input.ego_car_size[1],input.ego_car_size[2])

# 值的传递：
s=input.cross_starting_point_time2
g=input.cross_goal_point_time2
r=input.random_sample_area_cross_time2
rt=input.rengong_tuoyuan6
gk=input.ganzhi_kuang6
gt=input.ganzhi_tuoyuan6
zk=input.zhenzhi_kuang_cross 
# 值传递完毕

print(r)
tree=CCRRT.CcrrtGo(car=car, start=s,goal=g, ganzhi_kuang=gk, ganzhi_tuoyuan=gt, rand_area=r,rengong_tuoyuan=rt)
tree.p_safe=0.96
tree.max_n_node=5000
tree.planning(animation=False)
print(len(tree.path))
x=draw.Artist(rrt_object=tree,ganzhi_kuang=gk, zhenzhi_kuang=zk, tuoyuan=rt,if_draw_tree=0, is_clrrt=0)  # 坐标轴 area = [-25, 25, -25, 25]  # x-min x-max y-min y-max
x.play_cross()