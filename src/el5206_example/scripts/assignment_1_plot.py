#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math
import pandas as pd
import numpy as np

class OdomPlotter:
    def __init__(self):
        rospy.init_node('odom_plotter_node', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.ground_truth_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.ground_truth_callback)

        # Guardamos con timestamps
        self.odom_data = {'t': [], 'x': [], 'y': [], 'yaw': []}
        self.gt_data   = {'t': [], 'x': [], 'y': [], 'yaw': []}

    def ground_truth_callback(self, msg):
        t = msg.header.stamp.to_sec()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.gt_data['t'].append(t)
        self.gt_data['x'].append(x)
        self.gt_data['y'].append(y)
        self.gt_data['yaw'].append(yaw)

    def odom_callback(self, msg):
        t = msg.header.stamp.to_sec()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.odom_data['t'].append(t)
        self.odom_data['x'].append(x)
        self.odom_data['y'].append(y)
        self.odom_data['yaw'].append(yaw)

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def plot(self, df_sync):
        plt.plot(df_sync['x_odom'], df_sync['y_odom'], label='Odometry')
        plt.plot(df_sync['x_gt'], df_sync['y_gt'], label='Ground Truth')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Robot Trajectory (Synced by Time)')
        plt.axis('equal')
        plt.legend(loc="best")
        plt.grid()
        plt.show()

if __name__ == '__main__':
    plotter = OdomPlotter()
    rospy.spin()

    # Convertir a DataFrame
    df_odom = pd.DataFrame(plotter.odom_data)
    df_gt   = pd.DataFrame(plotter.gt_data)

    # Definir eje temporal común
    t_min = max(df_odom['t'].min(), df_gt['t'].min())
    t_max = min(df_odom['t'].max(), df_gt['t'].max())

    ts_common = np.union1d(df_odom['t'].values, df_gt['t'].values)
    ts_common = ts_common[(ts_common >= t_min) & (ts_common <= t_max)]

    # Reindexar con interpolación
    df_odom_sync = df_odom.set_index('t').reindex(ts_common).interpolate().reset_index()
    df_gt_sync   = df_gt.set_index('t').reindex(ts_common).interpolate().reset_index()

    # Combinar en un solo DataFrame
    df_sync = pd.DataFrame({
        't': ts_common,
        'x_odom': df_odom_sync['x'],
        'y_odom': df_odom_sync['y'],
        'yaw_odom': df_odom_sync['yaw'],
        'x_gt': df_gt_sync['x'],
        'y_gt': df_gt_sync['y'],
        'yaw_gt': df_gt_sync['yaw']
    })

    df_sync.to_csv('tabla_sync.csv', index=False)
    print("\nInterrupted. Plotting synced data...")
    plotter.plot(df_sync)
