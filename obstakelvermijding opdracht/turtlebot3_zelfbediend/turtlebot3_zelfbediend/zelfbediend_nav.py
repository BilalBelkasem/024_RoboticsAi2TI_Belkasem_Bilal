#!/usr/bin/env pythong3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

class ZelfbediendNav(Node):
    def __init__(self):
        super().__init__('zelfbediend_nav')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel',10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        self.subscription
        self.twitst = Twist()

    def laser_callback(self, msg):
        #Bepaal de afstand dat de robot detecteert een obstakel aan de voorkant
        front_distance = msg.ranges[len(msg.ranges) // 2-10: len(msg.ranges)// 2+10]
        min_distance = min(front_distance)

        if min_distance < 0.5:
        #controleert of er een obstakel is aan de voorkant van de robot
            self.twist.linear.x = 0.0 #stopt de robot
            self.twist.angular.z = random.choice([-1.0,1.0]) #draait de robot naar links of rechts random
        else:
            self.twist.x = 0.2
            self.twist.z = 0.0
        
        self.publisher_.publish(self.twist)

def main(args=None):
    #Om de ROS2-node te initialiseren laten draaien en ook afsluiten.
    rclpy.init(args=args)
    node = ZelfbediendNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown
if __name__ == '__main__':
    main()