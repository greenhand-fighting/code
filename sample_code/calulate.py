import numpy as np
# 给定一个树，计算路径的性质，最大风险，最小风险，平均风险，路径长度和路径点数，这五个数值。
class calulate_path_info:
    def __init__(self, tree_obj):
        tmp = [tree_obj.get_chance_constrain_from_pu(node) for node in tree_obj.path]  # 从这个可以看出这个finalpath里面都是一些节点
        self.path_min = np.min(tmp)
        self.path_max = np.max(tmp)
        self.path_avg = np.average(tmp)
        sum = 0
        for j in range(len(tree_obj.path) - 1):
            changdu, _ = tree_obj.calc_distance_and_angle(tree_obj.path[j], tree_obj.path[j + 1])
            sum = sum + changdu
            # 路径长度
        self.path_length=sum
        self.path_points_number=len(tree_obj.path)
