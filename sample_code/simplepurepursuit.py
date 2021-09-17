import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import math

k = 0.1    # 前视距离系数
Lfc = 2.0  # 前视距离
Kvp = 1.5  # 速度P控制器系数
Kpp = 0.1  # 位置P控制系数
dt = 0.05  # 时间间隔，单位：s
L = 2.9    # 车辆轴距，单位：m

class target_info:
    def __init__(self, t):
        self.x = t
        self.y = np.sin(t / 5.0) * t / 2.0
        self.vx = np.ones(len(t))
        self.vy = 0.1 * np.cos(t / 5.0) * t + np.sin(t / 5.0) / 2.0
    
class VehicleState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        

def update(state, a, delta):
    state.x = state.x + state.v * np.cos(state.yaw) * dt
    state.y = state.y + state.v * np.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * np.tan(delta) * dt
    state.v = state.v + a * dt
    
    
def PControl(target, current, distance):
    a = Kvp * (target - current) + Kpp * (distance - 5)
    return a

def calc_target_index(state, scanned_points_list, Lf):
    # 搜索最临近的路点
    dx = [state.x - icx for icx in scanned_points_list[0]]
    dy = [state.y - icy for icy in scanned_points_list[1]]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))

    Ld = d[ind]

    while Lf > Ld and (ind + 1) < len(scanned_points_list[0]):
        ind += 1
        Ld = d[ind]#math.sqrt(dx[ind] ** 2 + dy[ind] ** 2)
        
    del scanned_points_list[0][: ind]
    del scanned_points_list[1][: ind]
    return scanned_points_list[0][0], scanned_points_list[1][0]
    
def pure_pursuit_control(state, aim_point, Lf):
    alpha = math.atan2(aim_point[1] - state.y, aim_point[0] - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = np.pi - alpha
    delta = math.atan2(2.0 * L * np.sin(alpha) / Lf, 1.0)

    return delta
    
if __name__ == '__main__':
    t = np.arange(0, 50, 0.05)
    # 设置车辆的初始状态
    state = VehicleState(x=-0.0, y=-20.0, yaw=0.0, v=0.0)
    target_info = target_info(t)
    distance = math.sqrt((state.x - target_info.x[0]) ** 2 + (state.y - target_info.y[0]) ** 2)
    target_speed = np.sqrt(target_info.vx[0] ** 2 + target_info.vy[0] ** 2)
    
    #绘图部分
    fig = plt.figure()
    ax = fig.add_subplot()
    target_point, = ax.plot(target_info.x[0], target_info.y[0], "ro")
    target_line, = ax.plot(target_info.x[0], target_info.y[0], 'b')
    veh_point, = ax.plot(state.x, state.y, 'ko')
    veh_point_list = [[state.x], [state.y]]
    veh_line, = ax.plot(veh_point_list, color = 'gray')
    scanned_points_list = [[target_info.x[0]], [target_info.y[0]]]
    scanned_points, = ax.plot(scanned_points_list, color = 'gray', marker = 'o', linestyle = 'none')
    plt.grid(ls="--")

    # 开始制作动画
    def animate(i):
        target_point.set_xdata(target_info.x[i])
        target_point.set_ydata(target_info.y[i])
        target_line.set_xdata(target_info.x[:i])
        target_line.set_ydata(target_info.y[:i])

        global state
        Lf = k * state.v + Lfc
        
        if i % 10 == 0:
            global distance, target_speed
            distance = math.sqrt((state.x - target_info.x[i]) ** 2 + (state.y - target_info.y[i]) ** 2)
            target_speed = np.sqrt(target_info.vx[i] ** 2 + target_info.vy[i] ** 2)
            scanned_points_list[0].append(target_info.x[i])
            scanned_points_list[1].append(target_info.y[i])
            if len(scanned_points_list[0]) > 10:
                del scanned_points_list[0][: -10]
                del scanned_points_list[1][: -10]

        scanned_points.set_xdata(scanned_points_list[0])
        scanned_points.set_ydata(scanned_points_list[1])
            
        ai = PControl(target_speed, state.v, distance) #计算车辆与目标的速度和位置差并各自乘以一个比例系数
        aim_point = calc_target_index(state, scanned_points_list, Lf)
        di = pure_pursuit_control(state, aim_point, Lf)
        update(state, ai, di)
        veh_point.set_xdata(state.x)
        veh_point.set_ydata(state.y)
        veh_point_list[0].append(state.x)
        veh_point_list[1].append(state.y)
        veh_line.set_xdata(veh_point_list[0])
        veh_line.set_ydata(veh_point_list[1])
        if i == len(t) - 1:
            veh_point_list[0] = []
            veh_point_list[1] = []
            state = VehicleState(x=-0.0, y=-20.0, yaw=0.0, v=0.0)
            scanned_points_list[0] = [target_info.x[0]]
            scanned_points_list[1] = [target_info.y[0]]
    ani = animation.FuncAnimation(fig, animate, len(t), interval=1/dt, repeat=True, blit=False)
    ax.set_xlim([0, 50])
    ax.set_ylim([-30, 30])
    plt.show()
    #ani.save("pursuit.gif", writer='pillow')  #在当前目录下保存gif动态图片