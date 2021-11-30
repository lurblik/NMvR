import tkinter as tk
import numpy as np
from numpy.lib.function_base import angle
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from skimage import data, color, io, transform, img_as_bool
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from position import Position, Speed
from math import pow, atan2, sqrt, degrees, pi, cos, sin, radians, floor, ceil
window = tk.Tk()
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
        ## prejdena vzdialenost
        right = self.speed.lin_vel
        left = self.speed.lin_vel
        dc = (right + left) / 2
        # nova pozicia
        self.position.x += dc * cos(radians(self.position.theta))
        self.position.y += dc * sin(radians(self.position.theta))
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
    def __init__(self, x, y, axis):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'click', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.y = x
        self.x = y
        self.axis = axis
    def timer_callback(self):
        msg = String()
        msg.data = str(self.x) + "x" + str(self.y) + "y" + self.axis
        #self.get_logger().info("publishing %s" % msg.data)
        self.publisher.publish(msg)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.i = 0
        self.subscription
        self.positionPublisher = PositionPublisher()
        self.positionSubscriber = PositionSubscriber()
        self.position = Position(0, 0, 360)
        self.goalx = 0
        self.goaly = 0
        self.left = 0
        self.right = 0
    def euclidean_distance(self, update, dlzka=22):
        if update:
            self.update_position()
        result = sqrt(pow((int(self.goalx)/dlzka - self.position.x), 2) +
                    pow((int(self.goaly)/dlzka - self.position.y), 2))
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
        result = degrees(atan2((int(self.goaly)/len(pole)) - self.position.y, (int(self.goalx)/len(pole)) - self.position.x))
        if result < 0:
            result += 360
        return result
    def get_ang_vel(self, pole):
        angles = [0, 45, 90, 135, 180, 225, 270, 315, 360]
        angle_to_goal = np.argmin(angles, key=lambda x:x-abs(self.get_angle(pole)))
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
        self.goalx = float(input("Set goal X: "))
        self.goaly = float(input("Set goal Y: "))
        x = int(msg.data[0:msg.data.find('x')])
        y = int(msg.data[msg.data.find('x')+1:msg.data.find('y')])
        data = msg.data.replace('[', '')
        data = data.replace(']', '')
        data = data[data.find('y')+1:len(data)]
        pole = np.fromstring(data, dtype=int, sep=" ")
        pole = np.reshape(pole, (x, y))
        if self.i == 0:
            fig, ax = plt.subplots(figsize=(6, 6))
            createCanvas(fig)
            rclpy.spin_once(self.positionPublisher)
        else:
            fig = plt.gcf()
            plt.clf()
            plt.cla()
            ax = plt.gca()
            ax.clear()
            rclpy.spin_once(self.positionPublisher)
        ax.set_title("mapa")
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.pcolor(pole, cmap='Greys', edgecolors='k', linewidths=2)
        plt.gca().set_position([0, 0, 1, 1])
        angles = [0, 45, 90, 135, 180, 225, 270, 315, 360]
        positions = [0, 0.04545454545, 0.0909090909, 0.13636363636, 0.18181818181, 0.22727272727, 0.27272727272, 0.31818181818, 0.36363636363, 0.40909090909,\
        0.45454545454, 0.5, 0.54545454545, 0.5909090909, 0.63636363636, 0.68181818181, 0.72727272727, 0.77272727272, 0.81818181818, 0.86363636363, \
        0.90909090909, 0.95454545454, 1]
        
        while self.euclidean_distance(True) > 0.002:
            arrow = transform.rotate(io.imread('/home/nmvr/dev_ws/src/NMvR/NMvR/arrow.png'), self.position.theta)
            robo = fig.add_axes([(1/len(pole)), (1/len(pole)), (1/len(pole)), (1/len(pole))], anchor='C')
            robo.imshow(arrow)
            robo.axis('off')
            angle_diff = int(self.position.theta - self.get_angle(pole))
            print("difference", angle_diff)
            angle_diff = min(angles, key=lambda x:abs(x-abs(angle_diff)))
            if pole[int(self.position.y*22)][int(self.position.x*22)] == 1:
                print("CRASHED")
                break
            posX = min([k for k in positions if k-self.position.x>=-0.002])
            if self.position.theta+angle_diff == 180 or self.position.theta+angle_diff == 135 or self.position.theta+angle_diff == 225:
                posX -= 0.04545454545
            posY = min([k for k in positions if k-self.position.y>=-0.002])
            if self.position.theta+angle_diff == 270 or self.position.theta+angle_diff == 225 or self.position.theta+angle_diff == 315:
                posY -= 0.04545454545
            robo.set_position([posX,posY,(1/len(pole)),(1/len(pole))])
            ax.get_figure().canvas.draw()
            print("ANGLEDIFF", angle_diff)
            if angle_diff > 1 and angle_diff != 360:
                self.positionPublisher.speed.ang_vel = angle_diff
                self.positionPublisher.speed.lin_vel = 0
            else:
                self.positionPublisher.speed.ang_vel = 0
                self.positionPublisher.speed.lin_vel = self.linear_vel()
            print(self.i)
            self.i += 1
            rclpy.spin_once(self.positionPublisher)
            
        print("while")
        
def onclick(event):
    min_publisher = MinimalPublisher(int(event.xdata), int(event.ydata), event.inaxes.get_title())
    rclpy.spin_once(min_publisher)
    min_publisher.destroy_node()
def createCanvas(fig):
    canvas1 = FigureCanvasTkAgg(fig, master=window)
    canvas1.mpl_connect('button_press_event', onclick)
    canvas1.draw()
    canvas1.get_tk_widget().pack(side=tk.TOP)
def main(args=None):
    
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    while True:
        window.update()
        rclpy.spin_once(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
