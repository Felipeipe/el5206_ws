#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math
import pandas as pd
import numpy as np
import message_filters

class OdomPlotter:
    def __init__(self):
        rospy.init_node('odom_plotter_node', anonymous=True)

        odom_sub = message_filters.Subscriber('/odom', Odometry)
        gt_sub   = message_filters.Subscriber('/ground_truth/state', Odometry)

        # Sincronizador aproximado
        ats = message_filters.ApproximateTimeSynchronizer([odom_sub, gt_sub], 
                                                          queue_size=100, 
                                                          slop=0.1)  
        ats.registerCallback(self.sync_callback)

        self.data = {'t': [], 'x_odom': [], 'y_odom': [], 'yaw_odom': [],
                               'x_gt': [],   'y_gt': [],   'yaw_gt': []}

    def sync_callback(self, odom_msg, gt_msg):
        t = rospy.Time.now().to_sec()  # o usar odom_msg.header.stamp.to_sec()

        # Odom
        ox = odom_msg.pose.pose.position.x
        oy = odom_msg.pose.pose.position.y
        oq = odom_msg.pose.pose.orientation
        oyaw = self.quaternion_to_yaw(oq.x, oq.y, oq.z, oq.w)

        # Ground truth
        gx = gt_msg.pose.pose.position.x
        gy = gt_msg.pose.pose.position.y
        gq = gt_msg.pose.pose.orientation
        gyaw = self.quaternion_to_yaw(gq.x, gq.y, gq.z, gq.w)

        # Guardar sincronizado
        self.data['t'].append(t)
        self.data['x_odom'].append(ox)
        self.data['y_odom'].append(oy)
        self.data['yaw_odom'].append(oyaw)
        self.data['x_gt'].append(gx)
        self.data['y_gt'].append(gy)
        self.data['yaw_gt'].append(gyaw)

    def quaternion_to_yaw(self, x, y, z, w):
        import math
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


    def plot(self, df_sync):
        x_odom = df_sync['x_odom'].to_numpy(dtype=float)
        y_odom = df_sync['y_odom'].to_numpy(dtype=float)
        x_gt   = df_sync['x_gt'].to_numpy(dtype=float)
        y_gt   = df_sync['y_gt'].to_numpy(dtype=float)

        plt.plot(x_odom, y_odom, label='Odometry')
        plt.plot(x_gt, y_gt, label='Ground Truth')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Robot Trajectory (Synced by Time)')
        plt.axis('equal')
        plt.legend(loc="best")
        plt.grid()
        plt.show()

if __name__ == '__main__':
    plotter = OdomPlotter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    # Convertir todo lo guardado directamente a DataFrame
    df_sync = pd.DataFrame(plotter.data)

    # Guardar CSV
    df_sync.to_csv('tabla_sync.csv', index=False)

    print("\nInterrupted. Plotting synced data...")
    plotter.plot(df_sync)
