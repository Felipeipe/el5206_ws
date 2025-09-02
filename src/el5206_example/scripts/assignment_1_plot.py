#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import pandas as pd
import message_filters
import numpy as np
import math

class OdomPlotter:
    def __init__(self):
        rospy.init_node('odom_plotter_node', anonymous=True)

        odom_sub = message_filters.Subscriber('/odom', Odometry)
        gt_sub   = message_filters.Subscriber('/ground_truth/state', Odometry)

        rospy.Subscriber('/target_pose', Pose2D, self.goal_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        ats = message_filters.ApproximateTimeSynchronizer([odom_sub, gt_sub], 
                                                          queue_size=100, 
                                                          slop=0.1)  
        ats.registerCallback(self.sync_callback)

        self.params = {
            'k_att': rospy.get_param("/k_att"),
            'k_rep': rospy.get_param("/k_rep"),
            'rho0': rospy.get_param("/rho0"),
            'v_max': rospy.get_param("/v_max"),
            'w_max': rospy.get_param("/w_max"),
            'k_v': rospy.get_param("/k_v"),
            'k_w': rospy.get_param("/k_w"),
            'goal_tol': rospy.get_param("/goal_tol"),
            'slowdown_radius': rospy.get_param("/slowdown_radius"),
            'scan_downsample': rospy.get_param("/scan_downsample"),
            'rep_clip': rospy.get_param("/rep_clip"),
        }
        rospy.loginfo(self.params)

        self.data = {'t': [], 'x_odom': [], 'y_odom': [], 'yaw_odom': [],
                               'x_gt': [],   'y_gt': [],   'yaw_gt': []}
        self.goal = (0.0, 0.0)
        self.scan_points = []
        self.last_pose = (0.0, 0.0, 0.0)  # (x, y, yaw) del robot

    def sync_callback(self, odom_msg, gt_msg):
        t = rospy.Time.now().to_sec()  

        ox = odom_msg.pose.pose.position.x
        oy = odom_msg.pose.pose.position.y
        oq = odom_msg.pose.pose.orientation
        oyaw = self.quaternion_to_yaw(oq.x, oq.y, oq.z, oq.w)

        gx = gt_msg.pose.pose.position.x
        gy = gt_msg.pose.pose.position.y
        gq = gt_msg.pose.pose.orientation
        gyaw = self.quaternion_to_yaw(gq.x, gq.y, gq.z, gq.w)

        self.data['t'].append(t)
        self.data['x_odom'].append(ox)
        self.data['y_odom'].append(oy)
        self.data['yaw_odom'].append(oyaw)
        self.data['x_gt'].append(gx)
        self.data['y_gt'].append(gy)
        self.data['yaw_gt'].append(gyaw)

        # actualizo pose más reciente (para transformar el LIDAR)
        self.last_pose = (gx, gy, gyaw)

    def goal_callback(self, msg: Pose2D):
        self.goal = msg.x, msg.y

    def scan_callback(self, scan_msg: LaserScan):
        """Convierte el scan a puntos (x,y) en el marco global"""
        self.scan_points.clear()

        x_r, y_r, yaw_r = self.last_pose

        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
        for r, a in zip(scan_msg.ranges, angles):
            if np.isfinite(r):  # evitar inf
                # coordenadas en marco del robot
                xr = r * np.cos(a)
                yr = r * np.sin(a)

                # transformación a marco global
                xg = x_r + xr * np.cos(yaw_r) - yr * np.sin(yaw_r)
                yg = y_r + xr * np.sin(yaw_r) + yr * np.cos(yaw_r)

                self.scan_points.append((xg, yg))

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def plot(self, df_sync):
        x_gt   = df_sync['x_gt'].to_numpy(dtype=float)
        y_gt   = df_sync['y_gt'].to_numpy(dtype=float)
        goal_x, goal_y = self.goal 

        plt.figure(figsize=(8, 8))
        plt.plot(x_gt, y_gt, label='Ground Truth')
        plt.scatter(goal_x, goal_y, color='red', label='Goal Pose')

        # dibujar el scan ya en marco global
        if self.scan_points:
            xs, ys = zip(*self.scan_points)
            plt.scatter(xs, ys, s=5, c='gray', alpha=0.5, label='LaserScan (global)')

        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Robot Trajectory + LIDAR (global frame)')
        plt.axis('equal')
        plt.legend(loc="best")
        plt.grid()

        param_text = "\n".join([f"{k} = {v:.2f}" for k, v in self.params.items()])
        plt.gcf().text(1.02, 0.5, param_text, fontsize=10, va='center')
        plt.show()


if __name__ == '__main__':
    plotter = OdomPlotter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    df_sync = pd.DataFrame(plotter.data)
    print("\nInterrupted. Plotting synced data...")
    plotter.plot(df_sync)
