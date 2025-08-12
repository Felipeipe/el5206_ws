#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math

class OdomPlotter:
    def __init__(self):
        rospy.init_node('odom_plotter_node', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.ground_truth_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.ground_truth_callback)
        self.xs = []
        self.xt = []
        self.ys = []
        self.yt = []
        self.yaws = []
        self.yawt = []

    def ground_truth_callback(self, msg):
        # Extraer posición
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convertir orientación a yaw
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.xt.append(x)
        self.yt.append(y)
        self.yawt.append(yaw)

    def odom_callback(self, msg):
        # Extraer posición
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convertir orientación a yaw
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.xs.append(x)
        self.ys.append(y)
        self.yaws.append(yaw)

    def quaternion_to_yaw(self, x, y, z, w):
        # Fórmula estándar de conversión
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def plot(self):
        plt.plot(self.xs, self.ys, label='Odometry')
        plt.plot(self.xt, self.yt, label='Ground Truth')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Robot Trajectory from Odometry')
        plt.axis('equal')
        plt.legend(loc="best")
        plt.grid()
        plt.show()

if __name__ == '__main__':
    plotter = OdomPlotter()
    rospy.spin()
    print("\nInterrupted. Plotting...")
    plotter.plot()
