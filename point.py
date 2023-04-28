class Point:
    def __init__(self, x, y, weight=0, is_barrier=False):
        self.x = x
        self.y = y
        self.cost = 0
        self.parent = None
        self.weight = weight
        self.is_barrier = is_barrier
        self.area_new = self.get_point_area_new()   #增加一个属性表示节点所在的区域, 新区域函数
        self.area_old = self.get_point_area_old()   #增加一个属性表示节点所在的区域， 旧区域函数
        self.color = None
    def get_point_area_new(self):
        """
        根据坐标范围划分不同的区域，并为每个区域分配不同的权重
        """
        if self.x < 10 and self.y < 10:
            return 1+(10-((self.x + self.y) / (10 + 10)))
        elif self.x < 10 and self.y >= 10:
            return 5
        elif self.x >= 10 and self.y < 10:
            return 3
        else:
            return 4

    def get_point_area_old(self):
        """
        根据坐标范围划分不同的区域，并为每个区域分配不同的权重
        """
        if self.x < 10 and self.y < 10:
            return 0
        elif self.x < 10 and self.y >= 10:
            return 5
        elif self.x >= 10 and self.y < 10:
            return 3
        else:
            return 4