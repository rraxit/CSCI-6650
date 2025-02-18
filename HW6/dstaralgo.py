#python

import math
import numpy as np
from sys import maxsize
import sys
sys.path.append('/home/ridon/Downloads/HW4_cop/')
from Dstar import State

#Define constants
OBSTACLE_DIMENSIONS = [0.15, 0.15, 0.15]
OBSTACLE_HEIGHT = 1.0
MARKER_DIMENSIONS = [0.15, 0.15, 0.15]
START_MARKER_COLOR = [0, 1, 0]
END_MARKER_COLOR = [0, 0, 1]
MARKER_ELEVATION = 0.5
LINE_THICKNESS = 5
ROUTE_COLOR = [0.0, 0.0, 1.0]
MAX_LINES = 3500
ROUTE_ELEVATION = 0.15
quarten = np.array([0,0,0,1])



def sysCall_init():
    sim = require('sim')
    

# Functions for coordinate conversion

def calculate_preimage_coordinates(x, y): #get_preimage

  def reverse_transform(x):
    return (x - 50) / 20
    
  x_original = reverse_transform(x)
  y_original = reverse_transform(y)
  return x_original, y_original



class Map:
    
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list

    def get_neighbors(self, state):
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self, point_list, dilation=0):
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.map[x][y].set_state("#")
            if dilation > 0:
                for x_inc in range(dilation):
                    for y_inc in range(dilation):
                        try:
                            self.map[x+x_inc][y+y_inc].set_state("#")
                            self.map[x+x_inc][y-y_inc].set_state("#")
                            self.map[x-x_inc][y+y_inc].set_state("#")
                            self.map[x-x_inc][y-y_inc].set_state("#")
                        except: IndexError


class Dstar:
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()

    def process_state(self):
        x = self.min_state()

        if x is None:
            return -1

        k_old = self.get_kmin()
        self.remove(x)

        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        if k_old == x.h:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(x, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run(self, start, end):

        rx = []
        ry = []

        self.insert(end, 0.0)

        while True:
            self.process_state()
            if start.t == "close":
                break

        start.set_state("s")
        s = start
        s = s.parent
        s.set_state("e")
        tmp = start

        #AddNewObstacle(self.map) # add new obstacle after the first search finished

        while tmp != end:
            tmp.set_state("*")
            rx.append(tmp.x)
            ry.append(tmp.y)
            
            tmp = tmp.parent
        tmp.set_state("e")

        return rx, ry

    def modify(self, state):
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break

def AddNewObstacle(map:Map):
    ox, oy = [], []
    for i in range(5, 21):
        ox.append(i)
        oy.append(40)
    map.set_obstacle([(i, j) for i, j in zip(ox, oy)])



def build_map(ox, oy, start, goal):
    
    
    start_object_type = sim.primitiveshape_cuboid
    goal_object_type = sim.primitiveshape_cuboid
    
    start_handle = sim.createPrimitiveShape(start_object_type, MARKER_DIMENSIONS)
    goal_handle = sim.createPrimitiveShape(goal_object_type, MARKER_DIMENSIONS)
    
    x_preimg, y_preimg = calculate_preimage_coordinates(start[0], start[1])
    x_e_preimg, y_e_preimg = calculate_preimage_coordinates(goal[0], goal[1])
    
    sim.setObjectPosition(start_handle, -1, [x_preimg, y_preimg, MARKER_ELEVATION])
    sim.setObjectPosition(goal_handle, -1, [x_e_preimg, y_e_preimg, MARKER_ELEVATION])
    
    
def display_path(path_points):
    
    
    pimg_path_points = []
    # Convert route coordinates to simulation environment
    for i in range(path_points.shape[0]):
        pimg_path_points.append(list(calculate_preimage_coordinates(path_points[i][0], path_points[i][1])))
    
    pimg_path_points = np.array(pimg_path_points)
    
    path_object_handle = sim.addDrawingObject(sim.drawing_lines, LINE_THICKNESS, 0.0, -1, MAX_LINES, ROUTE_COLOR)
    
    for i, point in enumerate(pimg_path_points):
        if i > 0:
            last = pimg_path_points[i-1].tolist()
            last.append(ROUTE_ELEVATION)
            
            current = point.tolist()
            current.append(ROUTE_ELEVATION)
            
            last.extend(current)
            last.extend(current)
            last_x=last[:3]
            last_y=last[3:]
            last_x.extend(quarten)
            last_y.extend(quarten)
            last_y = last_y[3:]
            last_final = np.concatenate((last_x,last_y))
            sim.addDrawingObjectItem(path_object_handle, last)
    fixedVal = [ [1, 0,0,0,1  ] for _ in range(len(pimg_path_points))]
    fixedVal = np.array(fixedVal)
    pimg_path_points = np.column_stack((pimg_path_points, fixedVal)).flatten().tolist()
    
    path_f = sim.createPath(pimg_path_points)
    sim.setInt32Signal('path',path_f)
    pass


def sysCall_thread():
    # Set obstacle expansion
    dilation = 15
    
    # Initialize grid map
    m = Map(100, 100)
    
    # Define obstacles
    ox, oy = [], []
    
    # Define Borders
    for i in range(100):
      ox.append(i)
      oy.append(0)

    for i in range(100):
      ox.append(i)
      oy.append(100)

    for i in range(100):
        ox.append(100)
        oy.append(i)

    for i in range(100):
        ox.append(0)
        oy.append(i)


    # Additional obstacles
    for i in range(60):
        ox.append(30)
        oy.append(i)

    for i in range(30, 100):
        ox.append(60)
        oy.append(i)

    m.set_obstacle([(i, j) for i, j in zip(ox, oy)], dilation)

    
    # Define start and goal
    start = (15,20)
    goal = (90,90)
    
    build_map(ox, oy, start, goal)
    
    start = m.map[start[0]][start[1]]
    end = m.map[goal[0]][goal[1]]
    
    # Execute D-Star algorithm
    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)
    path_points = np.column_stack((rx, ry))
    
    display_path(path_points)