import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pandas as pd
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'click', self.listener_callback, 10)
        self.subscription
    def listener_callback(self, msg):
        if msg.data.find('mapa') != -1:
            #self.get_logger().info('I heard: "%s"' % msg.data)
            x = int(msg.data[0:msg.data.find('x')])
            y = int(msg.data[msg.data.find('x')+1:msg.data.find('y')])
            print(x, y)
            file = pd.read_csv('/home/nmvr/dev_ws/src/NMvR/NMvR/map.csv', delimiter=';', header=None)
            pole = file.to_numpy()
            if pole[x][y] == 1:
                pole[x][y] = 0
            else:
                pole[x][y] = 1
            data = pd.DataFrame(data=pole)
            data.to_csv('/home/nmvr/dev_ws/src/NMvR/NMvR/map.csv', sep=';', index=False, header=None)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()