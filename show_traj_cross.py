# generate a clinet
import carla
import random
import sys, os
import rospy
import math
client = carla.Client('localhost', 2000)
client.set_timeout(2.0)
world = client.get_world()

# --------------开始放置 自车  和 他车
transform = carla.Transform(carla.Location(x=-1.75, y=-13.0, z=1.0), carla.Rotation(yaw=90))
vehicles_transform = [carla.Transform(carla.Location(x = -5.3, y = 8.5, z = 1), carla.Rotation(yaw = 90)),
                            carla.Transform(carla.Location(x = -5.3, y = -9, z = 1), carla.Rotation(yaw = 90)),
                            carla.Transform(carla.Location(x = 1.4, y = -1.2, z = 1), carla.Rotation(yaw = 51)),
                            carla.Transform(carla.Location(x = 1.8, y = 8.5, z = 1), carla.Rotation(yaw = 270)),
                            carla.Transform(carla.Location(x = 6.4, y = 9.3, z = 1), carla.Rotation(yaw = 291)),
                            carla.Transform(carla.Location(x = 12, y = 1.8, z = 1), carla.Rotation(yaw = 0)),
                            carla.Transform(carla.Location(x = 20, y = 5.3, z = 1), carla.Rotation(yaw = 0))]
blueprints = world.get_blueprint_library().filter('vehicle')
blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
blueprints = [x for x in blueprints if not x.id.endswith('t2')]
blueprints = [x for x in blueprints if not x.id.endswith('coupe')]

for i in range(len(vehicles_transform)):
    blueprint=random.choice(blueprints)
    world.spawn_actor(blueprint, vehicles_transform[i])

blueprint_library = world.get_blueprint_library()
# 从浩瀚如海的蓝图中找到奔驰的蓝图
ego_vehicle_bp = blueprint_library.find('vehicle.mercedes-benz.coupe')
# 给我们的车加上特定的颜色
ego_vehicle_bp.set_attribute('color', '0, 255, 0')
print(transform)
ego_vehicle = world.spawn_actor(ego_vehicle_bp,  transform)
# --------------------自车 和 他车 放置完成


spectator = world.get_spectator()
transform = ego_vehicle.get_transform()
spectator.set_transform(carla.Transform(transform.location + carla.Location( z=39),
                                                    carla.Rotation(pitch=-90)))


# -----得到轨迹
data=[]
with open("trajcross.txt") as f:  # 打开文件
    lines = f.readlines()  # 读取文件
    for line in lines:
        line = line.strip('\n')
        line=line.split()
        line=[float(x) for x in line]
        data.append(line)
data=data[::-1]
#------得到轨迹  完成
print(transform)
print(data[0])
print(222222222222)
#-----绘制轨迹
debug = world.debug
draw_goal_point=0
for i in range(len(data)-1):
    jiaodu=math.degrees(data[i][2])
    
    x=data[i][0]
    y=data[i][1]
    x_=data[i+1][0] 
    y_=data[i+1][1]
    location=carla.Location( x, -y, z=0.1)
    location_=carla.Location(x_, -y_, z=0.1)
    if draw_goal_point % 2 ==0:
        debug.draw_box(carla.BoundingBox(location,carla.Vector3D(2.0,0.9,1.5)),carla.Rotation(yaw=-jiaodu-180), 0.1, carla.Color(0,255,0,0),0)
    draw_goal_point+=1
    debug.draw_arrow(location, location_, thickness=0.1, arrow_size=0.1, color=carla.Color(255,0,0))

debug.draw_point(carla.Location(x=6.5 ,y=4.7, z=1), size=0.05, color=carla.Color(255,0,0,0))
debug.draw_string(carla.Location(x=6.5 ,y=4.7, z=2), str('xxxxxxx' ), False, color=carla.Color(255,0,0),life_time=0)
    
#------绘制轨迹完成



