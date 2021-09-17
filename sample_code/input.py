import numpy as np

def generate_ganzhi_kuang(ganzhi_xinxi):
    ganzhi_kuang=[]
    for per in ganzhi_xinxi:
        ganzhi_kuang.append([per[0],per[1],per[2]/2,per[3]/2,per[4]])
    return ganzhi_kuang  #[x,y,length/2,width/2,yaw]
def generate_ganzhi_tuoyuan(ganzhi_xinxi):
    ganzhi_tuoyuan=[]
    for per in ganzhi_xinxi:
        x=[]
        x.append(per[0])
        x.append(per[1])
        x.append(per[5])
        x.append(per[6])
        x.append(per[4])
        x.append(per[5]-per[2]/2)
        x.append(per[6]-per[3]/2)
        ganzhi_tuoyuan.append(x)
    return ganzhi_tuoyuan  #椭圆（中心，半长轴，半短轴，yaw，sigma长轴，sigma短轴）

dis_threshold=25.5
def un_generate(dis, p1, p2):
    base = dis / dis_threshold
    sigma_base = np.abs(np.random.normal(0.0, base * p1))
    #print(sigma_base)
    return (base + sigma_base) * p2

def generate_rengong_tuoyuan(ganzhi_kuang, uncertainties):
    obs = []
    for gt, un in zip(ganzhi_kuang, uncertainties):
        a = gt[2]
        b = gt[3]
        d_a = a * (1 - b * np.sqrt((1 + np.tan(un[2])**2) / (b**2 + a**2 * np.tan(un[2])**2)))
        d_b = d_a / a * b
        obs.append((gt[0], gt[1], a + d_a + un[0], b + d_b + un[1], gt[4], d_a+un[0], d_b+un[1]))
    return obs

# 自车的形状：
ego_car_size=[4.51/2, 4.51/2, 2]   # 分别是前距离，后距离，车宽
# 起始点和终点   x值，y值，角度
starting_point=[12.25, 35, np.deg2rad(90.0)]
goal_point=[9.0, 70.0, np.deg2rad(90.0)]

starting_point_time2=[5.5, 50.7, np.deg2rad(90.0)]
goal_point_time2=[5.0, 85.0, np.deg2rad(90.0)]

starting_point_time3=[1.86, 61.3, np.deg2rad(90.0)]
goal_point_time3=[12.0, 95.0, np.deg2rad(90.0)]

starting_point_time4=[9.066, 67.66, np.deg2rad(90.0)]
goal_point_time4=[2.0, 100.0, np.deg2rad(90.0)]

#cross:
cross_starting_point_time1=[-1.75, 13.0, -np.pi/2]
cross_goal_point_time1=[4.5, -4, np.deg2rad(90.0)]

cross_starting_point_time2=[0.25, -2.0, -0.807054]
cross_goal_point_time2=[20.0, -1.75, np.deg2rad(90.0)]

# 采样区域
random_sample_area=[0, 15, 30, 72.5]   # x-min x-max y-min y-max
random_sample_area2=[0, 15, 45, 90]   # x-min x-max y-min y-max
random_sample_area3=[0, 15, 56, 100]   # x-min x-max y-min y-max
random_sample_area4=[0, 15, 62, 105]   # x-min x-max y-min y-max


random_sample_area_cross_time2=[-5, 25, -7, 0] 
random_sample_area_cross_time1=[-7, 7, -7, 16]
# 传感器信息
ganzhi_xinxi = [
            (5.58, 44.99, 4.12, 1.62, np.deg2rad(89.0), 2.7, 1.17),
            (5.47, 64.62, 4.4, 1.78, np.deg2rad(88.0), 2.87, 1.3),
            (9.17, 54.65, 4.17, 1.69, np.deg2rad(90.0), 2.65, 1.19),
            (12.67, 44.82, 3.88, 1.61, np.deg2rad(89.90), 2.49, 1.14),
            (12.76, 64.96, 3.95, 1.65, np.deg2rad(90.0), 2.91, 1.25),
        ]   # [x,y,长，宽，角度，长半轴，短半轴]

ganzhi_xinxi2= [
        (1.978, 74.55, 4.13, 1.63, np.deg2rad(90.0), 2.67, 1.2),
        (5.47, 64.54, 3.79, 1.63, np.deg2rad(89.0), 2.47, 1.14),
        (9.0, 54.93, 4.08, 1.59, np.deg2rad(90.0), 2.6, 1.14),
        (9.02, 84.86, 4.37, 1.67, np.deg2rad(90.0), 2.65, 1.32),
        (12.55, 64.96, 4.45, 1.71, np.deg2rad(90.0), 2.83, 1.18),
    ]
ganzhi_xinxi3= [
        (1.83, 74.57, 3.82, 1.59, np.deg2rad(90.0), 2.48, 1.13),
        (5.44, 65.03, 3.56, 1.54, np.deg2rad(89.0), 2.34, 1.11),
        (5.41, 94.42, 4.28, 1.65, np.deg2rad(90.0), 3.15, 1.37),
        (9.124, 84.98, 3.93, 1.66, np.deg2rad(90.0), 2.63, 1.23),
    ]
ganzhi_xinxi4= [
        (1.929, 74.62, 4.29, 1.6, np.deg2rad(90.0), 2.89, 1.21),
        (5.56, 94.53, 4.31, 1.66, np.deg2rad(90.0), 2.96, 1.27),
        (9.128, 84.93, 3.62, 1.64, np.deg2rad(90.0), 2.32, 1.14),
    ]

ganzhi_xinxi_cross1= [
        (1.361, 1.348, 3.53, 1.59, 2.237, 2.36,1.24),
        (-5.181, -8.498, 3.64, 1.61, 1.616,2.41,1.176),
        (1.6467, -8.446, 4.377,1.692, 1.5526,2.945, 1.259),
        (11.0285, -1.6743, 4.309,1.645, 3.109,2.899, 1.318),
        (6.3373, -9.1127, 4.0051,1.585, 1.36976, 2.928, 1.377),
        (-5.3691, 9.0414, 3.3907,1.5249, 1.5587, 2.2515, 1.1159)
    ]

ganzhi_xinxi_cross2=[
        (19.9126, -5.1657, 3.996,1.6033,3.09614,2.7425, 1.319),
        (6.3358, -9.1656, 4.2273,1.605, 1.3434,2.8204, 1.2810),
        (10.76, -1.9073, 3.712, 1.5643,3.112, 2.5059, 1.2517),
        (1.810, -8.54, 4.1776, 1.5893, 1.624,2.8743, 1.3125),
    ]
# 感知框
ganzhi_kuang=generate_ganzhi_kuang(ganzhi_xinxi)
ganzhi_kuang2=generate_ganzhi_kuang(ganzhi_xinxi2)
ganzhi_kuang3=generate_ganzhi_kuang(ganzhi_xinxi3)
ganzhi_kuang4=generate_ganzhi_kuang(ganzhi_xinxi4)
ganzhi_kuang5=generate_ganzhi_kuang(ganzhi_xinxi_cross1)
ganzhi_kuang6=generate_ganzhi_kuang(ganzhi_xinxi_cross2)
# 感知椭圆
ganzhi_tuoyuan=generate_ganzhi_tuoyuan(ganzhi_xinxi)
ganzhi_tuoyuan2=generate_ganzhi_tuoyuan(ganzhi_xinxi2)
ganzhi_tuoyuan3=generate_ganzhi_tuoyuan(ganzhi_xinxi3)
ganzhi_tuoyuan4=generate_ganzhi_tuoyuan(ganzhi_xinxi4)
ganzhi_tuoyuan5=generate_ganzhi_tuoyuan(ganzhi_xinxi_cross1)
ganzhi_tuoyuan6=generate_ganzhi_tuoyuan(ganzhi_xinxi_cross2)

# 人工椭圆
# 第一步 人工不确定性生成：
rengong_uncertainty = []
for obs in ganzhi_kuang:
    dist = np.hypot(starting_point[0] - obs[0], starting_point[1] - obs[1])
    dist=22
    un = (un_generate(dist, 0.5, 1.0),  # sigma_ver  纵向不确定性
          un_generate(dist, 0.3, 0.85),  # sigma_hor 横向不确定性
          un_generate(dist, 0.2, 0.1)  # sigma_radius 角度不确定性
          )
    rengong_uncertainty.append(un)
# 第二步：使用感知框和人工不确定性生成人工椭圆
rengong_tuoyuan=generate_rengong_tuoyuan(ganzhi_kuang,rengong_uncertainty)
rengong_tuoyuan2=generate_rengong_tuoyuan(ganzhi_kuang2,rengong_uncertainty)
rengong_tuoyuan3=generate_rengong_tuoyuan(ganzhi_kuang3,rengong_uncertainty)
rengong_tuoyuan4=generate_rengong_tuoyuan(ganzhi_kuang4,rengong_uncertainty)
rengong_tuoyuan5=generate_rengong_tuoyuan(ganzhi_kuang5,rengong_uncertainty)
rengong_tuoyuan6=generate_rengong_tuoyuan(ganzhi_kuang6,rengong_uncertainty)
# 人工椭圆生成完成


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
# def __init__(self, car, start, goal, obstacle_list, rand_area,obstacle_list_from_pu,obstacle_list_for_pengzhuang_jiance): # 车型，起点，终点，
# 类需要 车型，起止点，采样区域，感知框，感知椭圆，人工椭圆。
# 感知框，感知椭圆，人工椭圆都是从感知信息生成的
# 我的需要的修改的信息就是： 感知信息，真值框，起止点，车型，采样区域