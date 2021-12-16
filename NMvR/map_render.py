import tkinter as tk
from tkinter.constants import CENTER, END, NE, NW, W
import numpy as np
from numpy.core.records import array
from numpy.lib.function_base import angle
import pandas as pd
from matplotlib import colors, transforms
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from skimage import data, color, io, transform, img_as_bool
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
import heapq
#from position import Position, Speed
from math import pow, atan2, sqrt, degrees, pi, cos, sin, radians, floor, ceil
window = tk.Tk()
def setpole(arr):
    global pole
    pole = arr
def getpole():
    global pole
    return pole
def setgoalx(x):
    global goalx
    goalx = x
def getgoalx():
    global goalx
    return goalx
def setgoaly(y):
    global goaly
    goaly = y
def getgoaly():
    global goaly
    return goaly
class Position():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
class Speed():
    def __init__(self, lin_vel, ang_vel):
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
class PositionSubscriber(Node):
    def __init__(self):
        super().__init__('position_subscriber')
        self.pos_subscriber = self.create_subscription(String, 'position', self.timer_callback, 10)
        self.position = Position(0, 0, 360)
        self.speed = Speed(0, 0)
    def timer_callback(self, msg):
        self.speed.lin_vel = float(msg.data[0:msg.data.find('l')])
        self.speed.ang_vel = float(msg.data[msg.data.find('l')+1:msg.data.find('a')])
        ## nove natocenie
        self.position.theta += self.speed.ang_vel
        if self.position.theta > 360:
            self.position.theta -= 360
        #print("theta", self.position.theta)
        ## prejdena vzdialenost
        right = self.speed.lin_vel
        left = self.speed.lin_vel
        dc = (right + left) / 2
        # nova pozicia
        positions = list()
        for i in range(len(pole) + 1):
            positions.append(i*1/len(pole))
        self.position.x += dc * cos(radians(self.position.theta))
        self.position.x = min([k for k in positions if k-self.position.x>=-0.002])
        self.position.y += dc * sin(radians(self.position.theta))
        self.position.y = min([k for k in positions if k-self.position.y>=-0.002])
        
        #self.position.x = ceil(self.position.x)
        #self.position.y = ceil(self.position.y)
        #print(self.position.x, self.position.y)
class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.pos_publisher = self.create_publisher(String, 'position', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.speed = Speed(0, 0)
        self.i = 0
    def timer_callback(self):
        msg = String()
        msg.data = str(self.speed.lin_vel) + "l" + str(self.speed.ang_vel) + "a"
        self.get_logger().info("publishing %s" % msg.data)
        self.pos_publisher.publish(msg)
class MinimalPublisher(Node):
    def __init__(self, x, y, axis, value):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'click', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.y = x
        self.x = y
        self.axis = axis
        self.value = value
    def timer_callback(self):
        msg = String()
        msg.data = str(self.x) + "x" + str(self.y) + "y" + self.axis + str(self.value)
        self.get_logger().info("publishing %s" % msg.data)
        self.publisher.publish(msg)

class MinimalSubscriber(Node):
    def __init__(self, fig):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.i = 0
        self.subscription
        #self.positionPublisher = PositionPublisher()
        self.positionSubscriber = PositionSubscriber()
        self.position = Position(0, 0, 360)
        self.left = 0
        self.right = 0
        self.moving = False
        self.fig = fig
        array = [
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,0,0,0],
                [1,0,1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0],
                [1,1,1,1,0,0,0,1,1,1,1,1,1,1,0,0,0,1,1,1,0,0],
                [0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0],
                [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,0],
                [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0],
                [0,0,0,1,1,1,1,1,1,1,0,1,1,1,1,0,0,1,0,1,0,0],
                [0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,1,0,1,0,0],
                [0,0,1,1,1,1,1,1,0,1,0,0,0,0,1,0,0,1,0,1,0,0],
                [0,0,0,0,0,0,0,1,0,1,0,1,1,1,1,0,1,1,1,1,0,0],
                [0,0,0,0,0,0,0,1,0,1,0,1,0,0,1,1,0,1,0,1,0,0],
                [0,1,1,1,1,1,0,1,0,1,0,1,0,0,1,0,0,1,0,1,0,0],
                [0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,1,0,1,0,0],
                [0,1,0,0,1,1,1,1,1,1,1,0,1,1,1,0,0,1,0,1,0,0],
                [0,1,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,1,0,1,0,0],
                [0,1,0,0,1,0,0,0,0,0,1,0,0,0,1,0,0,1,0,1,0,0],
                [0,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,0,1,0,0],
                [0,1,0,0,1,0,0,1,0,0,1,0,1,0,0,0,0,0,0,0,0,0],
                [0,1,0,0,1,0,0,1,0,0,1,0,1,1,1,1,1,1,1,1,1,0],
                [0,1,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                ]
        setpole(array)
    def euclidean_distance(self, update, dlzka=22):
        if update:
            self.update_position()
        result = sqrt(pow((int(getgoalx())/dlzka - self.position.x), 2) +
                    pow((int(getgoaly())/dlzka - self.position.y), 2))
        #print("distance", result)
        return result
    
    def update_position(self):
        rclpy.spin_once(self.positionSubscriber)
        self.position = self.positionSubscriber.position
    def linear_vel(self, constant=0.3):
        result = self.euclidean_distance(False) * constant
        #print("linear", result)
        return result
    def get_angle(self, pole):
        result = degrees(atan2((int(getgoaly())/len(getpole())) - self.position.y, (int(getgoalx())/len(getpole())) - self.position.x))
        return result
    def get_ang_vel(self, pole):
        angles = [0, 45, 90, 135, 180, 225, 270, 315, 360]
        angle_to_goal = np.argmin(angles, key=lambda x:x-abs(self.get_angle(getpole())))
        angle_to_goal = self.position.theta - angle_to_goal
        print(self.get_angle(pole))
        print("ANGLE", angle_to_goal) 
        if angle_to_goal == 360 or angle_to_goal == 0:
            self.left = 0
            self.right = 0
        elif angle_to_goal == 45:
            self.left = 0
            self.right = 1*pi/4
        elif angle_to_goal == 90:
            self.left = 0
            self.right = 2*pi/4
        elif angle_to_goal == 135:
            self.left = 0
            self.right = 3*pi/4
        elif angle_to_goal == 180:
            self.left = 0
            self.right = 4*pi/4
        elif angle_to_goal == 225:
            self.left = 0
            self.right = 5*pi/4
        elif angle_to_goal == 270:
            self.left = 0
            self.right = 6*pi/4
        elif angle_to_goal == 315:
            self.left = 0
            self.right = 7*pi/4
        return angle_to_goal
    def check_around(self, pole, x, y):
        return [pole[y-1][x-1], pole[y-1][x], pole[y-1][x+1], pole[y][x-1],
        pole[y][x], pole[y][x+1], pole[y+1][x-1], pole[y+1][x], pole[y+1][x+1]] 
    def listener_callback(self, msg):
        # if not self.moving:
        #     self.goalx = float(input("Set goal X: "))
        #     self.goaly = float(input("Set goal Y: "))
        #     self.moving = True
        
        # x = int(msg.data[0:msg.data.find('x')])
        # y = int(msg.data[msg.data.find('x')+1:msg.data.find('y')])
        # data = msg.data.replace('[', '')
        # data = data.replace(']', '')
        # data = data[data.find('y')+1:len(data)]
        # data = np.fromstring(data, dtype=int, sep=" ")
        # setpole(np.reshape(data, (x, y)))
        # self.position.x = getgoalx()/len(getpole())
        # self.position.y = getgoaly()/len(getpole())
        fig = self.fig
        fig.clear()
        ax = plt.gca()
        ax.clear()
        mycmap = colors.ListedColormap(['white', 'black', 'yellow', 'green', 'blue','orange'])
        norm = colors.BoundaryNorm([0, .9, 1.9, 2.9, 3.9, 4.9, 28], mycmap.N)
        ax.pcolor(getpole(), cmap=mycmap, norm=norm, edgecolors='k', linewidths=2)
        ax.set_title("mapa")
        #print("positon", self.position.x, self.position.y)
        bbox = transforms.Bbox.from_extents(0,0,1,1)
        plt.gca().set_position(bbox)
        angles = [0, 45, 90, 135, 180, 225, 270, 315, 360]
        #rclpy.spin_once(self.positionPublisher)
        positions = list()
        for i in range(len(pole) + 1):
            positions.append(i*1/len(pole))
        #print("goal", getgoalx()/len(getpole()), getgoaly()/len(getpole()[0]))
        
        self.position.theta = self.get_angle(getpole())
        arrow = transform.rotate(io.imread('/home/nmvr/dev_ws/src/NMvR/NMvR/arrow.png'), self.position.theta)
        robo = fig.add_axes([(1/len(getpole())), (1/len(getpole())), (1/len(getpole())), (1/len(getpole()))], anchor='C')
        robo.imshow(arrow)
        robo.axis('off')
        self.position.x = getgoalx()/len(getpole())
        self.position.y = getgoaly()/len(getpole())
        
        robo.set_position([getgoalx()/len(getpole()),getgoaly()/len(getpole()[0]),(1/len(getpole())),(1/len(getpole()))])
        ax.get_figure().canvas.draw()
        

class okno(tk.Frame):
    def __init__(self, root):
        tk.Frame.__init__(self, root)
        rclpy.init(args=None)
        setgoaly(0)
        setgoalx(0)
        fig, ax = plt.subplots(figsize=(6, 6))
        minimal_subscriber = MinimalSubscriber(fig)
        rclpy.spin_once(minimal_subscriber)
        print("FIG", fig)
        ax.set_title("mapa")
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.pcolor(getpole(), cmap='Greys', edgecolors='k', linewidths=2)
        bbox = transforms.Bbox.from_extents(0,0,1,1)
        plt.gca().set_position(bbox)
        self.moving = True
        self.i = 0
        self.graph = {}
        ax.get_figure().canvas.draw()
        def onclick(event):
            print("click ", event)
            arr = getpole()
            if event.inaxes.get_title() == "mapa":
                if arr[int(event.ydata)][int(event.xdata)] == 1:
                    arr[int(event.ydata)][int(event.xdata)] = 0
                elif arr[int(event.ydata)][int(event.xdata)] == 0 or arr[int(event.ydata)][int(event.xdata)] > 1:
                    arr[int(event.ydata)][int(event.xdata)] = 1
            setpole(arr)
            rclpy.spin_once(minimal_subscriber)
            print("FIG2", fig)
            dstar()
        class Node:
            def __init__(self, id):
                self.id = id
                self.parents = {}
                self.children = {}
                self.g = float('inf')
                self.rhs = float('inf')
            def updateParents(self, parents):
                self.parents = parents
        
        def setmoving():
            self.moving = not self.moving
        def euclid(node1, node2):
            x = int(node1.split('x')[1][0]) - int(node2.split('x')[1][0])
            y = int(node1.split('y')[1][0]) - int(node2.split('y')[1][0])
            return x**2 + y**2
        def manhattan(node1, node2):
            return abs(node1[0]-node2[0]) + abs(node1[1]-node2[1])
        
        def stateNameToCoords(name):
            return [int(name.split('x')[1].split('y')[0]), int(name.split('x')[1].split('y')[1])]
        
        def lookAtNeighbors(node):
            neighbors = list()
            for i in range(-1, 2):
                for j in range(-1, 2):
                    if 0 <= node[0]+i < len(getpole()) and 0 <= node[1]+j < len(getpole()[0]):
                        neighbors.append('x'+str(node[0]+i)+'y'+str(node[1]+j))
            neighbors.remove('x'+str(node[0])+'y'+str(node[1]))
            return neighbors
        
        def addNodeToGraph(graph, id, neighbors, edge=1):
            node = Node(id)
            for i in neighbors:
                node.parents[i] = edge
                node.children[i] = edge
            graph[id] = node
            return graph
        
        def makeGraph(xsize, ysize):
            graph = {}
            for i in range(xsize):
                for j in range(ysize):
                    graph = addNodeToGraph(graph, 'x'+str(i)+'y'+str(j), lookAtNeighbors([i, j]))
            return graph
        
        def calculateKey(s, s_start):
            return [min(self.graph[s].g, self.graph[s].rhs + euclid(s, s_start)), min(self.graph[s].g, self.graph[s].rhs)]
        
        def updateVertex(queue, id, s_current, k_m):
            s_goal = self.goal
            if id != s_goal:
                min_rhs = float('inf')
                for i in self.graph[id].children:
                    min_rhs = min(
                        min_rhs, self.graph[i].g + self.graph[id].children[i])
                self.graph[id].rhs = min_rhs
            id_in_queue = [item for item in queue if id in item]
            if id_in_queue != []:
                if len(id_in_queue) != 1:
                    raise ValueError('more than one ' + id + ' in the queue!')
                queue.remove(id_in_queue[0])
            if self.graph[id].rhs != self.graph[id].g:
                heapq.heappush(queue, calculateKey(id, s_current) + [id])
        
        def computeShortestPath(queue, s_start, k_m):
            while (self.graph[s_start].rhs != self.graph[s_start].g) or (topKey(queue) < calculateKey(s_start, s_start)):
                k_old = topKey(queue)
                u = heapq.heappop(queue)[2]
                if k_old < calculateKey(u, s_start):
                    heapq.heappush(queue, calculateKey(u, s_start) + [u])
                elif self.graph[u].g > self.graph[u].rhs:
                    self.graph[u].g = self.graph[u].rhs
                    for i in self.graph[u].parents:
                        updateVertex(queue, i, s_start, k_m)
                else:
                    self.graph[u].g = float('inf')
                    updateVertex(queue, u, s_start, k_m)
                    for i in self.graph[u].parents:
                        updateVertex(queue, i, s_start, k_m)
        
        def topKey(queue):
            queue.sort()
            if len(queue) > 0:
                return queue[0][:2]
            else:
                return float('inf'), float('inf')
        
        def nextInShortestPath(s_current):
            min_rhs = float('inf')
            s_next = None
            if self.graph[s_current].rhs == float('inf'):
                print('STUCK')
            else:
                for i in self.graph[s_current].children:
                    child_cost = self.graph[i].g + self.graph[s_current].children[i]
                    if (child_cost) < min_rhs:
                        min_rhs = child_cost
                        s_next = i
                if s_next:
                    return s_next
                else:
                    raise ValueError('no children')
        
        def scanForObstacles(queue, s_current, scan_range, k_m):
            states_to_update = {}
            range_checked = 0
            if scan_range >= 1:
                for neighbor in self.graph[s_current].children:
                    neighbor_coords = stateNameToCoords(neighbor)
                    states_to_update[neighbor] = self.grid[neighbor_coords[1]][neighbor_coords[0]]
                range_checked = 1
        
            while range_checked < scan_range:
                new_set = {}
                for state in states_to_update:
                    new_set[state] = states_to_update[state]
                    for neighbor in self.graph[state].children:
                        if neighbor not in new_set:
                            neighbor_coords = stateNameToCoords(neighbor)
                            new_set[neighbor] = self.grid[neighbor_coords[1]][neighbor_coords[0]]
                range_checked += 1
                states_to_update = new_set
            new_obstacle = False
            for state in states_to_update:
                if states_to_update[state] == 1:  
                    for neighbor in self.graph[state].children:
                        if(self.graph[state].children[neighbor] != float('inf')):
                            neighbor_coords = stateNameToCoords(state)
                            self.graph[neighbor].children[state] = float('inf')
                            self.graph[state].children[neighbor] = float('inf')
                            updateVertex(queue, state, s_current, k_m)
                            new_obstacle = True
            return new_obstacle

        def moveAndRescan(queue, s_current, scan_range, k_m):
            if(s_current == self.goal):
                return 'goal', k_m
            else:
                s_last = s_current
                s_new = nextInShortestPath(s_current)
                new_coords = stateNameToCoords(s_new)
        
                if(self.grid[new_coords[1]][new_coords[0]] == 1):
                    s_new = s_current
        
                results = scanForObstacles(queue, s_new, scan_range, k_m)
                k_m += euclid(s_last, s_new)
                computeShortestPath(queue, s_current, k_m)
                return s_new, k_m
        
        def initDstarLite(queue, s_start, s_goal, k_m):
            queue = []
            self.graph[s_goal].rhs = 0
            heapq.heappush(queue, calculateKey(s_goal, s_start) + [self.goal])
            computeShortestPath(queue, s_start, k_m)
            return queue, k_m
        self.grid = getpole()
        self.goal = 'x1y4'
        self.s_start = 'x0y0'
        
        self.path = []
        self.graph = makeGraph(int(len(getpole())), int(len(getpole()[0])))
        self.i = 0
        def dstar():
            self.s_start = "x" + str(int(minimal_subscriber.position.x*len(getpole()))) + "y" + str(int(minimal_subscriber.position.y*len(getpole()[0])))
            self.grid = getpole()
            self.graph = makeGraph(int(len(getpole())), int(len(getpole()[0])))
            self.j = 0
            self.moving = True
            print("POLE", int(len(getpole())), int(len(getpole()[0])))
            k_m = 0
            queue = []
            queue, k_m = initDstarLite(queue, self.s_start, self.goal, k_m)
            done = False
            s_current = self.s_start
            pos_coords = stateNameToCoords(s_current)
            self.path = []
            while not done:
                s_new, k_m = moveAndRescan(queue, s_current, 10, k_m)
                self.path.append(s_new)
                if s_new == self.goal:
                    print('Goal Reached!')
                    done = True
                else:
                    s_current = s_new
                    print("S current", s_current)
                    pos_coords = stateNameToCoords(s_current)
            print(self.path)
            for i in range(len(self.path)-1):
                coords = stateNameToCoords(self.path[i])
                goalcoords = stateNameToCoords(self.path[len(self.path) - 1])
                arr = getpole()
                arr[coords[1]][coords[0]] = 2+self.i
                arr[goalcoords[1]][goalcoords[0]] = 20
                setpole(arr)
                rclpy.spin_once(minimal_subscriber)
            
            self.i += 1
        def newgoal():
            self.goal = "x" + str(xinput.get()) + "y" + str(yinput.get())
        def move():
            if self.j < len(self.path):
                if self.moving:
                    coords = stateNameToCoords(self.path[self.j])
                    setgoalx(coords[0])
                    setgoaly(coords[1])
                    print("GOAL", goalx, goaly)
                    rclpy.spin_once(minimal_subscriber)
                    self.j += 1
                    self.after(1, move)
            else:
                goalcoords = stateNameToCoords(self.path[len(self.path) - 1])
                arr = getpole()
                arr[goalcoords[1]][goalcoords[0]] = 0
                setpole(arr)
        print("CESTA", self.path)
        self.canvas1 = FigureCanvasTkAgg(fig, master=window)
        self.canvas1.mpl_connect('button_press_event', onclick)
        self.canvas1.get_tk_widget().place(relx=0.5, rely=0.5, anchor=CENTER)
        self.canvas1.draw()
        movebtn = tk.Button(self, text="Move", command = move)
        movebtn.place(relx=.9, rely=.9)
        drawbtn = tk.Button(self, text="Stop", command = setmoving)
        drawbtn.place(relx=.9, rely=.8)
        movebtn = tk.Button(self, text="D*Lite", command = dstar)
        movebtn.place(relx=.9, rely=.7)
        movebtn = tk.Button(self, text="New goal", command = newgoal)
        movebtn.place(relx=.9, rely=.6)
        xinput = tk.Entry(self)
        xinput.place(relx=.9, rely=.4)
        yinput = tk.Entry(self)
        yinput.place(relx=.9, rely=.5)
        self.pack(side="top", fill="both", expand=True)

def main(args=None):
    view = okno(window)
    view.pack(side="top", fill="both", expand=True)
    window.geometry("800x600")
    
    window.mainloop()
    #minimal_subscriber.destroy_node()
    
    
if __name__ == '__main__':
    main()
