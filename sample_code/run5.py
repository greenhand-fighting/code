import input, draw, calulate
import CCRRT, CLRRT, PURRT
import numpy as np
car=CCRRT.Vehicle(input.ego_car_size[0],input.ego_car_size[1],input.ego_car_size[2])

# 值的传递：
s=input.cross_starting_point_time1
g=input.cross_goal_point_time1
r=input.random_sample_area_cross_time1
rt=input.rengong_tuoyuan5
gk=input.ganzhi_kuang5
gt=input.ganzhi_tuoyuan5
zk=input.zhenzhi_kuang_cross
# 值传递完毕
# maxcc_list_100times=[]
# mincc_list_100times=[]
# avgcc_list_100times=[]
# long_list_100times = []
# pointnumber_list_100times = []
fail=0
for i in range(100):
    cc_rrt=CCRRT.CcrrtGo(car=car,start=s,goal=g, rand_area=r
                    ,rengong_tuoyuan=rt,ganzhi_tuoyuan=gt, ganzhi_kuang=gk)
    cc_rrt.planning(animation=False)
    #print("有多少路径 %f" % len(cc_rrt.path_end))
    #computer=calulate.calulate_path_info(cc_rrt)
    pengzhuang=0
    for node_in_final_path in cc_rrt.path:
        if cc_rrt.peng_zhuang_jian_ce(node_in_final_path, zk):  # 这个函数的参数第一个是节点，第二个是列表，列表是【障碍物x，障碍物y，长半轴，短半轴，yaw】！！！！！！！！！！！！！！！！！！！
            pengzhuang=1
            break
    if pengzhuang==0:
        # long_list_100times.append(computer.path_length)
        # maxcc_list_100times.append(computer.path_max)
        # mincc_list_100times.append(computer.path_min)
        # avgcc_list_100times.append(computer.path_avg)
        # pointnumber_list_100times.append(computer.path_points_number)
        pass
    else:
        fail+=1 
# pointnumber_list_100times=np.array(pointnumber_list_100times)
# long_list_100times=np.array(long_list_100times)
# maxcc_list_100times=np.array(maxcc_list_100times)
# mincc_list_100times=np.array(mincc_list_100times)
# avgcc_list_100times=np.array(avgcc_list_100times)
# print("max: %f" % np.average(maxcc_list_100times))
# print("avg: %f" % np.average(avgcc_list_100times))
# print("min: %f" % np.average(mincc_list_100times))
# print("points: %f" % np.average(pointnumber_list_100times))
# print("changdu: %f" % np.average(long_list_100times))
print('*'*100)
print("fail=:= %f" % fail)
success_rate = (100 - fail) / 100
print("ccrrt success1 rate: %f" % success_rate)

maxcc_list_100times=[]
mincc_list_100times=[]
avgcc_list_100times=[]
long_list_100times = []
pointnumber_list_100times = []
fail=0
succ=0
total_number=0
while succ!=100:
    #for i in range(100):
    #nt(i)
    cc_rrt=CCRRT.CcrrtGo(car=car,start=s,goal=g, rand_area=r
                    ,rengong_tuoyuan=rt,ganzhi_tuoyuan=gt, ganzhi_kuang=gk)
    cc_rrt.planning(animation=False)
  #  print("有多少路径 %f   fail : %d" % (len(cc_rrt.path_end),fail))
    computer=calulate.calulate_path_info(cc_rrt)
    pengzhuang=0
    for node_in_final_path in cc_rrt.path:
        if cc_rrt.peng_zhuang_jian_ce(node_in_final_path, zk):  # 这个函数的参数第一个是节点，第二个是列表，列表是【障碍物x，障碍物y，长半轴，短半轴，yaw】！！！！！！！！！！！！！！！！！！！
            pengzhuang=1
            break
    if pengzhuang==0 and len(cc_rrt.path_end):
        long_list_100times.append(computer.path_length)
        maxcc_list_100times.append(computer.path_max)
        mincc_list_100times.append(computer.path_min)
        avgcc_list_100times.append(computer.path_avg)
        pointnumber_list_100times.append(computer.path_points_number)
        succ+=1
        total_number+=1
        pass
    else:
        fail+=1 
        total_number+=1
pointnumber_list_100times=np.array(pointnumber_list_100times)
long_list_100times=np.array(long_list_100times)
maxcc_list_100times=np.array(maxcc_list_100times)
mincc_list_100times=np.array(mincc_list_100times)
avgcc_list_100times=np.array(avgcc_list_100times)
print("max: %f" % np.average(maxcc_list_100times))
print("avg: %f" % np.average(avgcc_list_100times))
print("min: %f" % np.average(mincc_list_100times))
print("points: %f" % np.average(pointnumber_list_100times))
print("changdu: %f" % np.average(long_list_100times))
print('*'*100)
print("fail=:= %f" % fail)
print("total_number=:= %f" % total_number)
# success_rate = (100 - fail) / 100
# print("ccrrt success2 rate: %f" % success_rate)

# maxcc_list_100times=[]
# mincc_list_100times=[]
# avgcc_list_100times=[]
# long_list_100times = []
# pointnumber_list_100times = []
fail=0
for i in range(100):
 #   print(i)
    cc_rrt=CLRRT.ClrrtGo(car=car,start=s,goal=g, rand_area=r
                    ,rengong_tuoyuan=rt,ganzhi_tuoyuan=gt, ganzhi_kuang=gk)
    cc_rrt.planning(animation=False)
  #  print("有多少路径 %f   fail : %d" % (len(cc_rrt.path_end),fail))
    #computer=calulate.calulate_path_info(cc_rrt)
    pengzhuang=0
    for node_in_final_path in cc_rrt.path:
        if cc_rrt.peng_zhuang_jian_ce(node_in_final_path, zk):  # 这个函数的参数第一个是节点，第二个是列表，列表是【障碍物x，障碍物y，长半轴，短半轴，yaw】！！！！！！！！！！！！！！！！！！！
            pengzhuang=1
            break
    if pengzhuang==0:
        # long_list_100times.append(computer.path_length)
        # maxcc_list_100times.append(computer.path_max)
        # mincc_list_100times.append(computer.path_min)
        # avgcc_list_100times.append(computer.path_avg)
        # pointnumber_list_100times.append(computer.path_points_number)
        pass
    else:
        fail+=1 
# pointnumber_list_100times=np.array(pointnumber_list_100times)
# long_list_100times=np.array(long_list_100times)
# maxcc_list_100times=np.array(maxcc_list_100times)
# mincc_list_100times=np.array(mincc_list_100times)
# avgcc_list_100times=np.array(avgcc_list_100times)
# print("max: %f" % np.average(maxcc_list_100times))
# print("avg: %f" % np.average(avgcc_list_100times))
# print("min: %f" % np.average(mincc_list_100times))
# print("points: %f" % np.average(pointnumber_list_100times))
# print("changdu: %f" % np.average(long_list_100times))
print('*'*100)
print("fail=:= %f" % fail)
success_rate = (100 - fail) / 100
print("clrrt success1 rate: %f" % success_rate)

maxcc_list_100times=[]
mincc_list_100times=[]
avgcc_list_100times=[]
long_list_100times = []
pointnumber_list_100times = []
fail=0
succ=0
total_number=0
while succ!=100:
#for i in range(100):
 #   print(i)
    cc_rrt=CLRRT.ClrrtGo(car=car,start=s,goal=g, rand_area=r
                    ,rengong_tuoyuan=rt,ganzhi_tuoyuan=gt, ganzhi_kuang=gk)
    cc_rrt.planning(animation=False)
 #   print("有多少路径 %f   fail : %d" % (len(cc_rrt.path_end),fail))
    computer=calulate.calulate_path_info(cc_rrt)
    pengzhuang=0
    for node_in_final_path in cc_rrt.path:
        if cc_rrt.peng_zhuang_jian_ce(node_in_final_path, zk):  # 这个函数的参数第一个是节点，第二个是列表，列表是【障碍物x，障碍物y，长半轴，短半轴，yaw】！！！！！！！！！！！！！！！！！！！
            pengzhuang=1
            break
    if pengzhuang==0 and len(cc_rrt.path_end):
        long_list_100times.append(computer.path_length)
        maxcc_list_100times.append(computer.path_max)
        mincc_list_100times.append(computer.path_min)
        avgcc_list_100times.append(computer.path_avg)
        pointnumber_list_100times.append(computer.path_points_number)
        succ+=1
        total_number+=1
        pass
    else:
        fail+=1 
        total_number+=1
pointnumber_list_100times=np.array(pointnumber_list_100times)
long_list_100times=np.array(long_list_100times)
maxcc_list_100times=np.array(maxcc_list_100times)
mincc_list_100times=np.array(mincc_list_100times)
avgcc_list_100times=np.array(avgcc_list_100times)
print("max: %f" % np.average(maxcc_list_100times))
print("avg: %f" % np.average(avgcc_list_100times))
print("min: %f" % np.average(mincc_list_100times))
print("points: %f" % np.average(pointnumber_list_100times))
print("changdu: %f" % np.average(long_list_100times))
print('*'*100)
print("fail=:= %f" % fail)
print("total_number=:= %f" % total_number)
# success_rate = (100 - fail) / 100
# print("clrrt success2 rate: %f" % success_rate)

# maxcc_list_100times=[]
# mincc_list_100times=[]
# avgcc_list_100times=[]
# long_list_100times = []
# pointnumber_list_100times = []
fail=0
for i in range(100):
 #   print(i)
    cc_rrt=CLRRT.ClrrtGo(car=car,start=s,goal=g, rand_area=r
                    ,rengong_tuoyuan=rt,ganzhi_tuoyuan=gt, ganzhi_kuang=gk)
    cc_rrt.planning(animation=False)
#    print("有多少路径 %f   fail : %d" % (len(cc_rrt.path_end),fail))
    #computer=calulate.calulate_path_info(cc_rrt)
    pengzhuang=0
    for node_in_final_path in cc_rrt.path:
        if cc_rrt.peng_zhuang_jian_ce(node_in_final_path, zk):  # 这个函数的参数第一个是节点，第二个是列表，列表是【障碍物x，障碍物y，长半轴，短半轴，yaw】！！！！！！！！！！！！！！！！！！！
            pengzhuang=1
            break
    if pengzhuang==0 and len(cc_rrt.path_end) and computer.path_max < 1-cc_rrt.p_safe:
        # long_list_100times.append(computer.path_length)
        # maxcc_list_100times.append(computer.path_max)
        # mincc_list_100times.append(computer.path_min)
        # avgcc_list_100times.append(computer.path_avg)
        # pointnumber_list_100times.append(computer.path_points_number)
        pass
    else:
        fail+=1 
# pointnumber_list_100times=np.array(pointnumber_list_100times)
# long_list_100times=np.array(long_list_100times)
# maxcc_list_100times=np.array(maxcc_list_100times)
# mincc_list_100times=np.array(mincc_list_100times)
# avgcc_list_100times=np.array(avgcc_list_100times)
# print("max: %f" % np.average(maxcc_list_100times))
# print("avg: %f" % np.average(avgcc_list_100times))
# print("min: %f" % np.average(mincc_list_100times))
# print("points: %f" % np.average(pointnumber_list_100times))
# print("changdu: %f" % np.average(long_list_100times))
print('*'*100)
print("fail=:= %f" % fail)
success_rate = (100 - fail) / 100
print("clrrt success3 rate: %f" % success_rate)

# maxcc_list_100times=[]
# mincc_list_100times=[]
# avgcc_list_100times=[]
# long_list_100times = []
# pointnumber_list_100times = []
fail=0
for i in range(100):
#    print(i)
    cc_rrt=PURRT.PurrtGo(car=car,start=s,goal=g, rand_area=r
                    ,rengong_tuoyuan=rt,ganzhi_tuoyuan=gt, ganzhi_kuang=gk)
    cc_rrt.planning(animation=False)
#    print("有多少路径 %f   fail : %d" % (len(cc_rrt.path_end),fail))
    #computer=calulate.calulate_path_info(cc_rrt)
    pengzhuang=0
    for node_in_final_path in cc_rrt.path:
        if cc_rrt.peng_zhuang_jian_ce(node_in_final_path, zk):  # 这个函数的参数第一个是节点，第二个是列表，列表是【障碍物x，障碍物y，长半轴，短半轴，yaw】！！！！！！！！！！！！！！！！！！！
            pengzhuang=1
            break
    if pengzhuang==0:
        # long_list_100times.append(computer.path_length)
        # maxcc_list_100times.append(computer.path_max)
        # mincc_list_100times.append(computer.path_min)
        # avgcc_list_100times.append(computer.path_avg)
        # pointnumber_list_100times.append(computer.path_points_number)
        pass
    else:
        fail+=1 
# pointnumber_list_100times=np.array(pointnumber_list_100times)
# long_list_100times=np.array(long_list_100times)
# maxcc_list_100times=np.array(maxcc_list_100times)
# mincc_list_100times=np.array(mincc_list_100times)
# avgcc_list_100times=np.array(avgcc_list_100times)
# print("max: %f" % np.average(maxcc_list_100times))
# print("avg: %f" % np.average(avgcc_list_100times))
# print("min: %f" % np.average(mincc_list_100times))
# print("points: %f" % np.average(pointnumber_list_100times))
# print("changdu: %f" % np.average(long_list_100times))
print('*'*100)
print("fail=:= %f" % fail)
success_rate = (100 - fail) / 100
print("purrt success1 rate: %f" % success_rate)


maxcc_list_100times=[]
mincc_list_100times=[]
avgcc_list_100times=[]
long_list_100times = []
pointnumber_list_100times = []
fail=0
succ=0
total_number=0
while succ!=100:
#for i in range(100):
#    print(i)
    cc_rrt=PURRT.PurrtGo(car=car,start=s,goal=g, rand_area=r
                    ,rengong_tuoyuan=rt,ganzhi_tuoyuan=gt, ganzhi_kuang=gk)
    cc_rrt.planning(animation=False)
#    print("有多少路径 %f   fail : %d" % (len(cc_rrt.path_end),fail))
    computer=calulate.calulate_path_info(cc_rrt)
    pengzhuang=0
    for node_in_final_path in cc_rrt.path:
        if cc_rrt.peng_zhuang_jian_ce(node_in_final_path, zk):  # 这个函数的参数第一个是节点，第二个是列表，列表是【障碍物x，障碍物y，长半轴，短半轴，yaw】！！！！！！！！！！！！！！！！！！！
            pengzhuang=1
            break
    if pengzhuang==0 and len(cc_rrt.path_end):
        long_list_100times.append(computer.path_length)
        maxcc_list_100times.append(computer.path_max)
        mincc_list_100times.append(computer.path_min)
        avgcc_list_100times.append(computer.path_avg)
        pointnumber_list_100times.append(computer.path_points_number)
        succ+=1
        total_number+=1
        pass
    else:
        fail+=1 
        total_number+=1
pointnumber_list_100times=np.array(pointnumber_list_100times)
long_list_100times=np.array(long_list_100times)
maxcc_list_100times=np.array(maxcc_list_100times)
mincc_list_100times=np.array(mincc_list_100times)
avgcc_list_100times=np.array(avgcc_list_100times)
print("max: %f" % np.average(maxcc_list_100times))
print("avg: %f" % np.average(avgcc_list_100times))
print("min: %f" % np.average(mincc_list_100times))
print("points: %f" % np.average(pointnumber_list_100times))
print("changdu: %f" % np.average(long_list_100times))
print('*'*100)
print("fail=:= %f" % fail)
print("total_number=:= %f" % total_number)
# success_rate = (100 - fail) / 100
# print("purrt success2 rate: %f" % success_rate)