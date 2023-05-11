# a_star.py
import sys
import numpy as np
from map import MAP
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches



class AStar:
    def __init__(self, map, heuristic_name):
        self.map = map
        self.open_set = []
        self.close_set = []
        self.parent = None
        self.start_point = None
        self.end_point = None
        self.heuristic_name = heuristic_name



    @staticmethod
    def time_decay_factor(start_time, current_time, decay_rate):
        elapsed_time = current_time - start_time
        decay_factor = max(5 - decay_rate * elapsed_time, 0)
        return decay_factor



    # 计算当前点的历史行驶距离
    def BaseCost(self, p):
        x_dis = abs(p.x - self.start_point.x)
        y_dis = abs(p.y - self.start_point.y)
        return x_dis + y_dis

    # 启发式函数-新
    def HeuristicCost_new(self, p):
        x_dis = abs(p.x - self.end_point.x)
        y_dis = abs(p.y - self.end_point.y)
        return min(x_dis, y_dis)

    # 启发式函数-曼哈顿距离
    def HeuristicCost_ManD(self, p):
        x_dis = abs(p.x - self.end_point.x)
        y_dis = abs(p.y - self.end_point.y)
        return x_dis + y_dis

    # 启发式函数-欧式距离
    def HeuristicCost_EucD(self, p):
        x_dis = abs(p.x - self.end_point.x) ** 2
        y_dis = abs(p.y - self.end_point.y) ** 2
        dis = np.sqrt(x_dis + y_dis)
        return dis

    # 启发式函数-切比雪夫距离
    def HeuristicCost_CheD(self, p):
        x_dis = abs(p.x - self.end_point.x)
        y_dis = abs(p.y - self.end_point.y)
        return max(x_dis, y_dis)

    # 启发式函数-对角线距离
    def HeuristicCost_DiaD(self, p):
        x_dis = abs(p.x - self.end_point.x)
        y_dis = abs(p.y - self.end_point.y)
        dis = np.sqrt(x_dis + y_dis)
        return dis

    # 启发式函数-加权曼哈顿距离
    def HeuristicCost_WManD(self, p):
        x_dis = 1.1 * abs(p.x - self.end_point.x)
        y_dis = 1.2 * abs(p.y - self.end_point.y)
        return max(x_dis, y_dis)

    # 启发式函数-加权欧式距离
    def HeuristicCost_WEucD(self, p):
        x_dis = 1.1 * abs(p.x - self.end_point.x) ** 2
        y_dis = 1.2 * abs(p.y - self.end_point.y) ** 2
        dis = np.sqrt(x_dis + y_dis)
        return dis

    def is_valid_and_not_in_close_list(self, x, y, close_set):
        if self.IsValidPoint(x, y):
            point = self.map.point_list[x][y]
            if not point.is_barrier and not self.IsInCloseList(point):
                return True
        return False

    def HeuristicCost_h1(self, p):
        x_dis = abs(p.x - self.end_point.x)
        y_dis = abs(p.y - self.end_point.y)
        dis = x_dis + y_dis
        return dis

    def HeuristicCost_h2(self, p, close_set):
        if p.parent is None:
            return 99999

        next_x, next_y = None, None

        for i in range(-1, 2):
            for j in range(-1, 2):
                if self.is_valid_and_not_in_close_list(p.x + i, p.y + j, close_set):
                    next_x = p.x + i  # 下一个节点的横坐标
                    next_y = p.y + j  # 下一个节点的纵坐标
                    break

        if next_x is None or next_y is None:
            return 99999

        x_dis = (p.parent.x - p.x) * (p.x - next_x) + (p.parent.y - p.y) * (p.y - next_y)
        y_dis = np.sqrt((p.parent.x - p.x) ** 2 + (p.parent.y - p.y) ** 2) + np.sqrt((p.x - next_x) ** 2 + (p.y - next_y) ** 2)
        dis = x_dis / y_dis
        return dis



    def TotalCost(self, p):
        # 总成本
        current_time = time.time()
        decay_factor = AStar.time_decay_factor(self.start_time, current_time, 0.1)
        # weight = p.weight * 10
        # if self.heuristic_name == "Hn":
        #     return self.BaseCost(p) + self.HeuristicCost_new(p) + weight
        # if self.heuristic_name == "Man":
        #     return self.BaseCost(p) + self.HeuristicCost_ManD(p) + weight
        # if self.heuristic_name == "Euc":
        #     return self.BaseCost(p) + self.HeuristicCost_EucD(p) + weight
        # if self.heuristic_name == "Che":
        #     return self.BaseCost(p) + self.HeuristicCost_CheD(p) + weight
        # if self.heuristic_name == "Dia":
        #     return self.BaseCost(p) + self.HeuristicCost_DiaD(p) + weight
        # if self.heuristic_name == "WMan":
        #     return self.BaseCost(p) + self.HeuristicCost_WManD(p) + weight
        # if self.heuristic_name == "WEuc":
        #     return self.BaseCost(p) + self.HeuristicCost_WEucD(p) + weight
        # if self.heuristic_name == "H2":
        #     return self.BaseCost(p) + 2 * self.HeuristicCost_h2(p, self.close_set) + 2 * self.HeuristicCost_h1(p) + weight

        weight = p.weight * 10
        if self.heuristic_name == "Hn":

           # return self.BaseCost(p) + self.HeuristicCost_new(p) + weight + p.area_new * 10 # 动态规划，添加时间因子作为权重的一部分。
           # return self.BaseCost(p) + self.HeuristicCost_new(p) + weight
           return  p.area_old * 0.2 * self.BaseCost(p) + self.HeuristicCost_new(p) + weight

        if self.heuristic_name == "Man":
            # return self.BaseCost(p) + self.HeuristicCost_ManD(p) + weight + p.area_new * 10
            return p.area_old * 0.2 * self.BaseCost(p) + self.HeuristicCost_ManD(p) + weight
        if self.heuristic_name == "Euc":
            # return self.BaseCost(p) + self.HeuristicCost_EucD(p) + weight + p.area_new * 10
            return p.area_old * 0.2 * self.BaseCost(p) + self.HeuristicCost_EucD(p) + weight
        if self.heuristic_name == "Che":
            # return self.BaseCost(p) + self.HeuristicCost_CheD(p) + weight + p.area_new * 10
            return p.area_old * 0.2 * self.BaseCost(p) + self.HeuristicCost_CheD(p) + weight
        if self.heuristic_name == "Dia":
            return  p.area_old * 0.2 * self.BaseCost(p) + self.HeuristicCost_DiaD(p) + weight
            # return p.area_old * 10 * self.BaseCost(p) + 3 * self.HeuristicCost_h2(p, self.close_set) + 3 * self.HeuristicCost_h1(p) + weight
        if self.heuristic_name == "WMan":
            # return self.BaseCost(p) + self.HeuristicCost_WManD(p) + weight + p.area_new * 10
            return p.area_old * 0.2 * self.BaseCost(p) + self.HeuristicCost_WManD(p) + weight
        if self.heuristic_name == "WEuc":
            # return self.BaseCost(p) + self.HeuristicCost_WEucD(p) + weight + p.area_new * 10
            return p.area_old * 0.2 * self.BaseCost(p) + self.HeuristicCost_WEucD(p) + weight
        if self.heuristic_name == "H2":
            # return self.BaseCost(p) + self.HeuristicCost_WEucD(p) + weight + p.area_new * 10
            return p.area_old * 0.2 * self.BaseCost(p) + 2 * self.HeuristicCost_h2(p, self.close_set) + 2 * self.HeuristicCost_h1(p) + weight

    # 判断点是否有效
    def IsValidPoint(self, x, y):
        if x < 0 or y < 0 or x >= X_MAX_LEN or y >= Y_MAX_LEN:
            return False
        if self.map.IsObstacle(x, y):
            return False
        return True

    # 判断点是否在open-set
    def IsInPointList(self, p, point_list):
        for point in point_list:
            if point.x == p.x and point.y == p.y:
                return True
        return False

    # 判断点是否在open-set
    def IsInOpenList(self, p):
        return self.IsInPointList(p, self.open_set)

    # 判断点是否在close-set
    def IsInCloseList(self, p):
        return self.IsInPointList(p, self.close_set)

    def IsStartPoint(self, p):
        return p.x == self.start_point.x and p.y == self.start_point.y

    def IsEndPoint(self, p):
        return p.x == self.end_point.x and p.y == self.end_point.y

    #
    def ProcessPoint(self, x, y, parent):
        if not self.IsValidPoint(x, y):
            return  # Do nothing for invalid point
        p = self.map.point_list[x][y]
        if self.IsInCloseList(p):
            return  # Do nothing for visited point
        if not self.IsInOpenList(p):
            p.parent = parent
            p.cost = self.TotalCost(p)
            p.area_old = p.get_point_area_old() # mobile area design
            p.area_new = p.get_point_area_new() # mobile area design
            self.open_set.append(p)

    def SelectPointInOpenList(self):
        index = 0
        selected_index = -1
        min_cost = sys.maxsize
        for p in self.open_set:
            cost = self.TotalCost(p)
            if cost < min_cost:
                min_cost = cost
                selected_index = index
            index += 1
        return selected_index

    # 获取求得路径
    def BuildPath(self, p):
        path = []
        while True:
            path.insert(0, (p.x, p.y))  # Insert first
            if self.IsStartPoint(p):
                break
            else:
                p = p.parent
        return path

    def set_start_point(self, x, y):
        if self.map.point_list[x][y].is_barrier:
            print("出发点和到达点请从以下列表中选择:")
            point_list = []
            for i in range(self.map.x_len):
                for j in range(self.map.y_len):
                    if self.map.point_list[i][j].is_barrier:
                        continue
                    point_list.append([i, j])
            print(point_list)
            assert False
        self.start_point = self.map.point_list[x][y]

    def set_end_point(self, x, y):
        if self.map.point_list[x][y].is_barrier:
            print("出发点和到达点请从以下列表中选择:")
            point_list = []
            for i in range(self.map.x_len):
                for j in range(self.map.y_len):
                    if self.map.point_list[i][j].is_barrier:
                        continue
                    point_list.append([i, j])
            print(point_list)
            assert False
        self.end_point = self.map.point_list[x][y]



    # 路径规划主入口

    def find_one_path(self):
        self.start_time = time.time()

        self.open_set = []
        self.close_set = []

        self.open_set.append(self.start_point)
        while True:
            index = self.SelectPointInOpenList()
            if index < 0:
                print('No path found, algorithm failed!!!')
                return
            p = self.open_set[index]


            if self.IsEndPoint(p):
                return self.BuildPath(p)

            del self.open_set[index]
            self.close_set.append(p)


            # Process all neighbors
            x = p.x
            y = p.y
            # 4 adjacency search
            # for step in [[0, 1], [0, -1],
            #              [1, 0], [-1, 0]]:
            # 8 adjacency search
            for step in [[0, 1], [0, -1],
                         [1, 0], [-1, 0],
                         [1, 1], [-1, -1],
                         [1, -1], [-1, 1]]:
            # 16 adjacency search
            # for step in [[0, 1], [0, -1],
            #              [1, 0], [-1, 0],
            #              [1, 1], [-1, -1],
            #              [1, -1], [-1, 1],
            #              [2, 1], [-2, -1],  # add 8 new direction
            #              [2, -1], [-2, 1],
            #              [1, 2], [-1, -2],
            #              [-1, 2], [1, -2]]:

                self.ProcessPoint(x + step[0], y + step[1], p)

            # self.update_plot(map_handle, p, self.open_set, self.heuristic_name)
    def check_path(self, one_path):
        for point in one_path:
            if self.map.point_list[point[0]][point[1]].is_barrier:
                print(point)
                return False
        return True


if __name__ == "__main__":
    # 地图最大长度，宽度
    X_MAX_LEN = 20
    Y_MAX_LEN = 20
    map_handle = MAP(x_len=X_MAX_LEN, y_len=Y_MAX_LEN)

    # Wheelchair people and Visually Imparied People 20x20 map1
    # 设置障碍点以及权重
    # map_handle.add_barrier_point(7, 9, 2)
    # map_handle.add_barrier_point(12, 1, 2)
    # map_handle.add_barrier_point(18, 9, 1)
    # map_handle.add_barrier_point(4, 5, 7)
    # map_handle.add_barrier_point(7, 8, 7)
    # map_handle.add_barrier_point(8, 15, 7)
    # map_handle.add_barrier_point(10, 14, 2)
    # map_handle.add_barrier_point(17, 16, 2)
    # map_handle.add_barrier_point(0, 3, 1)
    # map_handle.add_barrier_point(0, 4, 1)
    # map_handle.add_barrier_point(0, 5, 1)
    # map_handle.add_barrier_point(0, 6, 1)
    # map_handle.add_barrier_point(1, 11, 3)
    # map_handle.add_barrier_point(2, 13, 3)
    # map_handle.add_barrier_point(5, 5, 3)
    # map_handle.add_barrier_point(10, 16, 6)

    # Wheelchair people 20x20 map2
    # for x in range(3, 7):
    #     for y in range(4):
    #         map_handle.add_barrier_point(x, y, 2)
    # for x in range(10, 15):
    #     for y in range(4, 9):
    #         map_handle.add_barrier_point(x, y, 1)
    #
    # # 设置从(6, 16)至(6, 12)的垂直障碍物
    # for y in range(12, 17):
    #     map_handle.add_barrier_point(6, y, 5)
    #
    # # 设置从(6, 12)至(9, 12)的水平障碍物
    # for x in range(6, 10):
    #     map_handle.add_barrier_point(x, 12, 5)
    #
    # map_handle.add_barrier_point(7, 9, 2)
    # map_handle.add_barrier_point(12, 1, 2)
    # map_handle.add_barrier_point(18, 9, 1)
    # map_handle.add_barrier_point(4, 5, 7)
    # map_handle.add_barrier_point(7, 8, 7)
    # map_handle.add_barrier_point(8, 15, 7)
    # map_handle.add_barrier_point(10, 14, 2)
    # map_handle.add_barrier_point(17, 16, 2)
    # map_handle.add_barrier_point(0, 3, 1)
    # map_handle.add_barrier_point(0, 4, 1)
    # map_handle.add_barrier_point(0, 5, 1)
    # map_handle.add_barrier_point(0, 6, 1)
    # map_handle.add_barrier_point(1, 11, 3)
    # map_handle.add_barrier_point(2, 13, 3)
    # map_handle.add_barrier_point(5, 5, 3)
    # map_handle.add_barrier_point(10, 16, 6)


    # Hearing Imparied people 20x20 map3
    for x in range(3, 7):
        for y in range(4):
            map_handle.add_barrier_point(x, y, 5)
    for x in range(10, 15):
        for y in range(4, 9):
            map_handle.add_barrier_point(x, y, 6)
    for x in range(6, 11):
        for y in range(12, 17):
            map_handle.add_barrier_point(x, y, 4)

    map_handle.add_barrier_point(7, 9, 1)
    map_handle.add_barrier_point(12, 1, 1)
    map_handle.add_barrier_point(18, 9, 1)
    map_handle.add_barrier_point(4, 5, 1)
    map_handle.add_barrier_point(7, 8, 1)
    map_handle.add_barrier_point(8, 15, 1)
    map_handle.add_barrier_point(10, 14, 1)
    map_handle.add_barrier_point(17, 16, 1)
    map_handle.add_barrier_point(0, 3, 1)
    map_handle.add_barrier_point(0, 4, 1)
    map_handle.add_barrier_point(0, 5, 1)
    map_handle.add_barrier_point(0, 6, 1)
    map_handle.add_barrier_point(1, 11, 1)
    map_handle.add_barrier_point(2, 13, 1)
    map_handle.add_barrier_point(5, 5, 1)

    map_handle.update_barrier_round()
    # 遍历所有的启发式方法
    for name in ['Hn', 'Man', 'Euc', 'Che', 'Dia', 'WMan', 'WEuc', 'H2']:
        # astar_handle = AStar(map=map_handle, heuristic_name='Hn')
        # astar_handle = AStar(map=map_handle, heuristic_name='Man')
        # astar_handle = AStar(map=map_handle, heuristic_name='Euc')
        # astar_handle = AStar(map=map_handle, heuristic_name='Che')
        # astar_handle = AStar(map=map_handle, heuristic_name='Dia')
        astar_handle = AStar(map=map_handle, heuristic_name='H2')
        # 设置起点 终点
        start_point = [0, 0]
        end_point = [12, 12]
        astar_handle.set_start_point(start_point[0], start_point[1])
        astar_handle.set_end_point(end_point[0], end_point[1])
        map_handle.add_start_point(start_point[1], start_point[0])
        map_handle.add_end_point(end_point[1], end_point[0])
        # 计算路径
        start_time = time.time()
        one_path = astar_handle.find_one_path()
        assert astar_handle.check_path(one_path)
        end_time = time.time()
        # elapsed_time = end_time - start_time
        # print("Heuristic name:", 'Hn', "path len:", len(one_path))
        # print("Heuristic name:", 'Man', "path len:", len(one_path))
        # print("Heuristic name:", 'Euc', "path len:", len(one_path))
        # print("Heuristic name:", 'Che', "path len:", len(one_path))
        # print("Heuristic name:", 'Dia', "path len:", len(one_path))
        print("Heuristic name:", 'H2', "path len:", len(one_path))
        print(one_path)
        elapsed_time = end_time - start_time
        print("Algorithm running time:", elapsed_time, "s")
        # 画图
        map_handle.add_path(one_path)
        # map_handle.plot(_name='Hn', path = one_path)
        # map_handle.plot(_name='Man', path = one_path)
        # map_handle.plot(_name='Euc', path = one_path)
        # map_handle.plot(_name='Che', path = one_path)
        # map_handle.plot(_name='Dia',path = one_path)
        map_handle.plot(_name='H2', path = one_path)

        break
