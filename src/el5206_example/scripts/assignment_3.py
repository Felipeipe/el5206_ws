#!/usr/bin/env python3
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import random

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# ---------- Auxiliary Functions ----------
def yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)

def rot2d(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s],
                     [s,  c]])

def line_from_points(p1, p2):
    (x1, y1), (x2, y2) = p1, p2
  # COMPLETAR con la ecuación de la recta de toda la vida
  ## Recta a*x + b*y + c = 0 normalizada (c >= 0) a partir de 2 puntos.

    a = (y1 - y2) / (x2 -x1)
    b = 1
    c = -a*x1-y1
    norm = math.hypot(a, b)
    if norm == 0:
        return None
    a, b, c = a/norm, b/norm, c/norm
    if c < 0:  # consistencia de signos
        a, b, c = -a, -b, -c
    return (a, b, c)

# ---------- Nodo principal ----------
class GeometricMapperNode:
    def __init__(self):
        rospy.init_node("assignment3_mapper_no_refit", anonymous=True)

        # Params
        self.duration = rospy.get_param("~duration", 15.0)      # s de grabación
        self.sample_rate = rospy.get_param("~sample_rate", 0.5) # Hz de muestreo
        self.min_range = rospy.get_param("~min_range", 0.05)
        self.max_range = rospy.get_param("~max_range", 10.0)

        # RANSAC (sin fusión posterior, sin refit)
        self.ransac_dist_thresh = rospy.get_param("~ransac_dist_thresh", 0.05)
        self.ransac_min_inliers = rospy.get_param("~ransac_min_inliers", 100)
        self.ransac_max_iter = rospy.get_param("~ransac_max_iter", 1000)
        self.max_lines = rospy.get_param("~max_lines", 5)

        # Subs
        var_odom = "/ground_truth/state" # /odom
        self.sub_odom = rospy.Subscriber(var_odom, Odometry, self.odom_cb, queue_size=10)
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.scan_cb, queue_size=10)

        # Buffers
        self.last_odom = None 
        self.last_scan = None
        self.global_pts = [] # Puntos en coordenadas GLOBALES

        # Timer
        self.start_time = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(1.0/self.sample_rate), self.sample_cb)

        rospy.loginfo("Assignment 3: Grabando...")

    # Callbacks para que usen  estas variables directamente
    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.last_odom = (x, y, yaw)

    def scan_cb(self, msg):
        self.last_scan = msg

    # ---- Muestreo periódico ----
    def sample_cb(self, _evt):
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed >= self.duration:
            self.timer.shutdown()
            rospy.loginfo("Grabación terminada (%.1fs). Procesando...", elapsed)
            self.finish_and_plot()
            return

        if self.last_odom is None or self.last_scan is None:
            return

        # 1) puntos LOCALES
        local_pts = self.scan_to_local_points(self.last_scan, self.min_range, self.max_range)
        if not local_pts:
            return

        # 2) Transformación de locales a globales
        x, y, yaw = self.last_odom
        T_bo = self.make_SE2(x, y, yaw)
        glob = self.apply_SE2_to_points(T_bo, local_pts)
        self.global_pts.extend(glob)

    # ---- Geometría ----
    def scan_to_local_points(self, scan, rmin, rmax):
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        pts = []
        for rng, ang in zip(scan.ranges, angles):
            if np.isfinite(rng) and rmin <= rng <= rmax:
                pts.append((rng*math.cos(ang), rng*math.sin(ang)))
        return pts

    def make_SE2(self, tx, ty, th):
        R = rot2d(th)
        T = np.eye(3)
        T[:2, :2] = R
        T[:2, 2] = [tx, ty]
        return T

    def apply_SE2_to_points(self, T, pts):
        if not pts:
            return []
        P = np.c_[np.array(pts), np.ones((len(pts), 1))].T  # 3xN
        G = T @ P
        return list(map(tuple, G[:2, :].T))

    # ---- RANSAC  ----
    def ransac_lines(self, points, dist_thresh, min_inliers, max_iter, max_lines):
    # COMPLETAR
        lines = []
        point_1 = random.choice(points)
        point_2 = random.choice(points)
        line = line_from_points(point_1, point_2)
        for i in range(max_iter):
            inliers = []
            for p in points:
                distance = abs(line[0]*p[0] + line[1]*p[1] + line[2])
                if distance < dist_thresh:
                    inliers.append(p)
            if len(inliers) >= min_inliers:
                lines.append(line)
                if len(lines) >= max_lines:
                    break
                points = [p for p in points if p not in inliers]
            point_1 = random.choice(points)
            point_2 = random.choice(points)
            line = line_from_points(point_1, point_2)
        return lines

    # ---- Graficar ----
    def finish_and_plot(self):
        if not self.global_pts:
            rospy.logwarn("No se recolectaron puntos. ¿/scan entrega datos?")
            return

        lines = self.ransac_lines(self.global_pts,
                                  self.ransac_dist_thresh,
                                  self.ransac_min_inliers,
                                  self.ransac_max_iter,
                                  self.max_lines)

        rospy.loginfo("Líneas detectadas (sin refit): %d", len(lines))

        # Límites del gráfico a partir de la nube de puntos
        gx, gy = zip(*self.global_pts)
        xmin, xmax = min(gx)-1.0, max(gx)+1.0
        ymin, ymax = min(gy)-1.0, max(gy)+1.0

        plt.figure(figsize=(8, 8))

        # nube de puntos tenue para contexto
        plt.scatter(gx, gy, s=1, alpha=0.15, label='LIDAR (global)')

        # Dibujar cada línea de RANSAC
        for (a, b, c) in lines:
            if abs(b) > 1e-6:
                xs = np.linspace(xmin, xmax, 2)
                ys = (-a*xs - c) / b
            else:
                ys = np.linspace(ymin, ymax, 2)
                xs = (-b*ys - c) / a
            plt.plot(xs, ys, '-', linewidth=2.0,
                     label='Línea' if 'Línea' not in plt.gca().get_legend_handles_labels()[1] else None)

        plt.axis('equal'); plt.xlim([xmin, xmax]); plt.ylim([ymin, ymax])
        plt.xlabel('X [m]'); plt.ylabel('Y [m]')
        plt.title('Assignment 3: Líneas (sin trayectoria, sin fusión, sin refit)')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.show()

# ---------- Main ----------
if __name__ == "__main__":
    try:
        node = GeometricMapperNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
