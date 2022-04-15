import numpy as np
import math
import random
import copy

class Node():
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None

class RRTPlanner:
    def __init__(self,cube,radius):
        self.cube = cube
        self.radius = radius
    def planning(self,obs_x,obs_y,start_x,start_y,tar_x,tar_y,minx,miny,maxx,maxy):
        self.start = Node(start_x,start_y)
        self.end = Node(tar_x,tar_y)
        self.expandis = 0.2
        self.getgoal = 0.1
        self.obs = np.array([obs_x,obs_y]).T
        self.nodelist = [self.start]

        while (True):
            if random.random() > self.getgoal:
                rnd = self.random_node()
            else :
                rnd = [self.end.x,self.end.y]
            min_index = self.get_nearest(self.nodelist,rnd)
            nearest_node = self.nodelist[min_index]
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
            new_node = copy.deepcopy(nearest_node)
            new_node.x += self.expandis * math.cos(theta)
            new_node.y += self.expandis * math.sin(theta)
            new_node.parent = min_index

            if not self.check_collision(new_node):
                continue
            if new_node.x < minx or new_node.x > maxx or new_node.y < miny or new_node.y > maxy:
                continue 
            self.nodelist.append(new_node)

            dx = new_node.x - self.end.x
            dy = new_node.y - self.end.y
            d = math.sqrt(dx ** 2 + dy ** 2)
            if d <= self.expandis:
                break;
        path = [[self.end.x,self.end.y]]
        last_index = len(self.nodelist) - 1
        while self.nodelist[last_index].parent is not None:
            node = self.nodelist[last_index]
            path.append([node.x,node.y])
            last_index = node.parent
        path.append([self.start.x,self.start.y])
        path.reverse()
        path = np.array(path)
        return (path[:,0],path[:,1])

    def random_node(self):
        node_x = random.uniform(-10, 10)
        node_y = random.uniform(-10, 10)
        node = [node_x,node_y]
        return node
    def get_nearest(self,node_list,rnd):
        dis_list = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in node_list]
        min_index = dis_list.index(min(dis_list))
        return min_index
    def check_collision(self,new_node):
        flag = 1
        for (ox,oy) in self.obs:
            dx = ox - new_node.x
            dy = oy - new_node.y
            d = math.sqrt(dx ** 2 + dy ** 2)
            if (d <= self.radius):
                flag = 0
        return flag
    





