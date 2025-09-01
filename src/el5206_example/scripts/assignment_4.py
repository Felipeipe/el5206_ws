#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import math
import time
import yaml
import numpy as np
import matplotlib.pyplot as plt

import rospy
import rospkg
import tf
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class EL5206_Robot:
    def __init__(self):
        # ---------- ROS Node ----------
        rospy.init_node('EL5206_Assignment4', anonymous=False)

        # ---------- Frames ----------
        self.robot_frame_id = "base_link"
        self.odom_frame_id  = 'world'  # 'odom'

        # ---------- Estados ----------
        self.currentScan = None
        self.odom_x = self.odom_y = self.odom_yaw = None
        self.target_x = self.target_y = self.target_yaw = None

        # ---------- Paths ----------
        self.path = rospkg.RosPack().get_path('el5206_example') if rospkg.RosPack() else "."

        # ---------- Parámetros (ajustar) ----------
        self.k_att  = rospy.get_param("~k_att", 1.0)        # Ganancia atractiva
        self.k_rep  = rospy.get_param("~k_rep", 0.5)        # Ganancia repulsiva
        self.rho0   = rospy.get_param("~rho0", 0.8)         # Umbral de influencia de obstáculos (m)
        self.v_max  = rospy.get_param("~v_max", 0.4)        # Velocidad lineal máxima (m/s)
        self.w_max  = rospy.get_param("~w_max", 1.0)        # Velocidad angular máxima (rad/s)
        self.k_v    = rospy.get_param("~k_v", 0.8)          # Ganancia para v ~ ||F||*cos(phi)
        self.k_w    = rospy.get_param("~k_w", 1.5)          # Ganancia para w ~ heading(F)
        self.goal_tol = rospy.get_param("~goal_tol", 0.10)  # Tolerancia de posición (m)
        self.slowdown_radius = rospy.get_param("~slowdown_radius", 0.8)  # Reduce v cerca de meta
        self.scan_downsample = rospy.get_param("~scan_downsample", 2)    # Usar 1 de cada N rayos
        self.rep_clip = rospy.get_param("~rep_clip", 5.0)   # Clip de cada término repulsivo

        # ---------- ROS I/O ----------
        rospy.Subscriber("/ground_truth/state", Odometry, self.odometryCallback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scanCallback, queue_size=1)
        rospy.Subscriber("/target_pose", Pose2D, self.poseCallback, queue_size=1)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # ---------- Aux ----------
        self.rate_hz = rospy.get_param("~rate", 20.0)
        self.rate = rospy.Rate(self.rate_hz)

        rospy.loginfo("Nodo de Campos Potenciales iniciado...")

    # ======================= Callbacks =======================

    def scanCallback(self, msg: LaserScan):
        self.currentScan = msg

    def odometryCallback(self, msg: Odometry):
        self.odom_x, self.odom_y, self.odom_yaw = self.odom2Coords(msg)

    def poseCallback(self, msg: Pose2D):
        # Meta a alcanzar
        self.target_x   = float(msg.x)
        self.target_y   = float(msg.y)
        self.target_yaw = float(msg.theta)
        rospy.loginfo("Nueva meta recibida: x=%.3f, y=%.3f, th=%.3f",
                      self.target_x, self.target_y, self.target_yaw)

    # ======================= Utilidades =======================

    def odom2Coords(self, odom_msg: Odometry):
        """
        Devuelve (x, y, yaw) desde el mensaje de odometría (en marco odom).
        """
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        return x, y, yaw

    def world_to_robot(self, vx, vy, yaw):
        """
        Rota un vector del marco mundo al marco del robot (rota por -yaw).
        """
        cy, sy = math.cos(-yaw), math.sin(-yaw)
        rx = cy * vx - sy * vy
        ry = sy * vx + cy * vy
        return rx, ry

    # ======================= Core APF =======================

    def compute_attractive(self):
        """
        Fuerza atractiva en el marco del robot.
        F_att = k_att * R(-yaw) * [dx, dy]
        """
        if self.target_x is None or self.odom_x is None:
            return np.zeros(2)

        dx = self.target_x - self.odom_x
        dy = self.target_y - self.odom_y
        ex_r, ey_r = self.world_to_robot(dx, dy, self.odom_yaw)
        F_att = self.k_att * np.array([ex_r, ey_r])
        return F_att

    def compute_repulsive(self):
        """
        Fuerza repulsiva en el marco del robot sumando contribuciones de /scan.
        Modelo clásico:
            F_rep_i = k_rep * (1/ρ - 1/ρ0) * (1/ρ^2) * (-ê_rayo)
        para 0 < ρ < ρ0, 0 en otro caso.
        ê_rayo apunta hacia el obstáculo, por eso va con signo negativo.
        """
        if self.currentScan is None:
            return np.zeros(2)

        ranges = np.array(self.currentScan.ranges)
        a_min  = self.currentScan.angle_min
        a_inc  = self.currentScan.angle_increment
        r_min  = self.currentScan.range_min
        r_max  = self.currentScan.range_max

        # Downsample para bajar ruido/costo
        idxs = np.arange(0, len(ranges), max(1, int(self.scan_downsample)))

        F_rep = np.zeros(2)
        for i in idxs:
            rho = ranges[i]
            if not np.isfinite(rho):
                continue
            if rho < r_min or rho > r_max:
                continue
            if rho <= 0.0 or rho >= self.rho0:
                continue

            theta = a_min + i * a_inc  # ángulo del rayo en el marco del robot
            # Dirección hacia el obstáculo (desde el robot): ê = [cos, sin]
            e = np.array([math.cos(theta), math.sin(theta)])

            # Magnitud del potencial
            mag = self.k_rep / rho**2
            # Dirección de la fuerza es opuesta al obstáculo:
            f_i = -mag * e

            # Clip para evitar explosiones numéricas
            f_i = np.clip(f_i, -self.rep_clip, self.rep_clip)
            F_rep += f_i

        return F_rep

    def forces_to_cmd(self, F):
        """
        Convierte la fuerza resultante F (en marco del robot) a (v, w).
        - Dirección deseada: phi = atan2(F_y, F_x)
        - w = k_w * phi (saturado)
        - v = k_v * ||F|| * cos(phi), >= 0 (saturado y reducido cerca de meta)
        """
        Fx, Fy = F[0], F[1]
        normF = np.linalg.norm(F)
        if normF < 1e-6:
            return 0.0, 0.0

        phi = math.atan2(Fy, Fx)

        # Angular
        w = self.k_w * phi
        w = max(-self.w_max, min(self.w_max, w))

        # Lineal: avanza más cuando F apunta hacia +x (cos positivo)
        v_nom = self.k_v * normF * math.cos(phi)

        # Slowdown cerca de la meta
        if self.target_x is not None and self.odom_x is not None:
            dx = self.target_x - self.odom_x
            dy = self.target_y - self.odom_y
            dist = math.hypot(dx, dy)
            if dist < self.slowdown_radius:
                # Escala linealmente hasta 0 en la meta
                v_nom = v_nom * (dist / self.slowdown_radius)

        v = max(0.0, min(self.v_max, v_nom))
        
        return v, w

    # ======================= Loop principal =======================

    def assignment_4(self):
        """
        Bucle de control de Campos Potenciales.
        Corre hasta que se detenga el nodo.
        """
        rospy.loginfo("Empezó el mambo...")
        while not rospy.is_shutdown():
            twist = Twist()

            # Esperar a datos mínimos
            if self.odom_x is None or self.currentScan is None or self.target_x is None:
                self.vel_pub.publish(twist)  # stop
                self.rate.sleep()
                continue

            # Meta alcanzada???
            dx = self.target_x - self.odom_x
            dy = self.target_y - self.odom_y
            dist = math.hypot(dx, dy)
            if dist <= self.goal_tol:
                # Parar suave
                self.vel_pub.publish(Twist())
                self.rate.sleep()
                continue

            # Fuerzas
            F_att = self.compute_attractive()
            F_rep = self.compute_repulsive()
            F = F_att + F_rep

            # Comando
            v, w = self.forces_to_cmd(F)
            twist.linear.x  = v
            twist.angular.z = w
            self.vel_pub.publish(twist)

            self.rate.sleep()


if __name__ == '__main__':
    node = EL5206_Robot()
    rospy.loginfo("Nodo EL5206 Assignment 4 listo.")
    try:
        node.assignment_4()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception")
