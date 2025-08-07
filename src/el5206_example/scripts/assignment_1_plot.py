#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math

class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter_node')
        self.odom_sub = self.create_subscription(Odometry, 'rellena el tópico aquí', self.odom_callback, 10)
        
        self.xs = []
        self.ys = []
        self.yaws = []

    def odom_callback(self, msg):
        # Completar la posición
        x = 
        y = 
        # Orientación -> yaw

        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.xs.append(x)
        self.ys.append(y)
        self.yaws.append(yaw)

        self.get_logger().info(f'Pose: x={x:.2f}, y={y:.2f}, θ={math.degrees(yaw):.2f}°')

    def quaternion_to_yaw(self, x, y, z, w):
        # Completar la conversión de quaternion a yaw
        return math.atan2(siny_cosp, cosy_cosp)

    def plot(self):
        plt.plot(self.xs, self.ys, label='Odometry')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Robot Trajectory from Odometry')
        plt.axis('equal')
        plt.legend()
        plt.grid()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = OdomPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterrupted. Plotting...")
        node.plot()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
