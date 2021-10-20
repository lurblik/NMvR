import tkinter as tk
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from skimage import data, color, io, transform, img_as_bool
import rclpy
from std_msgs.msg import String
from rclpy.node import Node
window = tk.Tk()

class MinimalPublisher(Node):
    def __init__(self, x, y, axis):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'click', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.y = x
        self.x = y
        self.axis = axis
    def timer_callback(self):
        msg = String()
        msg.data = str(self.x) + "x" + str(self.y) + "y" + self.axis
        self.get_logger().info("publishing %s" % msg.data)
        self.publisher.publish(msg)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'topic', self.listener_callback, 10)
        self.i = 0
        self.subscription
    def listener_callback(self, msg):
        x = int(msg.data[0:msg.data.find('x')])
        y = int(msg.data[msg.data.find('x')+1:msg.data.find('y')])
        data = msg.data.replace('[', '')
        data = data.replace(']', '')
        #print(data)
        data = data[data.find('y')+1:len(data)]
        pole = np.fromstring(data, dtype=int, sep=" ")
        pole = np.reshape(pole, (x, y))
        if self.i == 0:
            fig, ax = plt.subplots(figsize=(6, 6))
            createCanvas(fig)
        else:
            fig = plt.gcf()
            plt.clf()
            plt.cla()
            ax = plt.gca()
            ax.clear()
        ax.set_title("mapa")
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.pcolor(pole, cmap='Greys', edgecolors='k', linewidths=2)
        plt.gca().set_position([0, 0, 1, 1])
        arrow = io.imread('/home/nmvr/dev_ws/src/NMvR/NMvR/arrow.png')
        robo = fig.add_axes([(1/len(pole)), (1/len(pole)), (1/len(pole)), (1/len(pole))], anchor='C')
        robo.imshow(arrow)
        robo.axis('off')
        ax.get_figure().canvas.draw()
        print(self.i)
        self.i += 1
        
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
