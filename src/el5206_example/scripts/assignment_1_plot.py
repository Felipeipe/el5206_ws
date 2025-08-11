#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import math
import threading

class OdomPlotter:
    def __init__(self, duration=30):
        rospy.init_node('odom_vs_gt_plotter_node', anonymous=True)

        # Suscriptores
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.gt_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.gt_callback)

        # Datos odometría estimada
        self.odom_xs, self.odom_ys = [], []
        # Datos ground truth
        self.gt_xs, self.gt_ys = [], []

        self.duration = duration

        rospy.loginfo(f"OdomPlotter iniciado. Grabando datos por {self.duration} segundos...")

        # Temporizador para detener después del tiempo indicado
        threading.Timer(self.duration, self.stop_and_plot).start()

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_xs.append(x)
        self.odom_ys.append(y)

    def gt_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.gt_xs.append(x)
        self.gt_ys.append(y)

    def stop_and_plot(self):
        rospy.loginfo("Tiempo completado. Generando gráfico...")
        rospy.signal_shutdown("Fin de grabación")
        self.plot()

    def plot(self):
        plt.figure()
        plt.plot(self.odom_xs, self.odom_ys, label='Odometry (/odom)', linewidth=2)
        plt.plot(self.gt_xs, self.gt_ys, label='Ground Truth (/ground_truth/state)', linewidth=2, linestyle='--')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Comparación de Trayectorias')
        plt.axis('equal')
        plt.legend()
        plt.grid()
        plt.show()

if __name__ == '__main__':
    try:
        OdomPlotter(duration=30)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
