#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
import math
import numpy as np

class MoveRobotNode:
    def __init__(self):
        rospy.init_node("move_robot_node", anonymous=True)

        self.approach = 2 # o 2

        # Publishers / Subscribers
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.target_callback)

        # Variables de estado
        self.current_pose = Pose2D()
        self.target_pose = None

        rospy.loginfo(f"Node started. Using approach {self.approach}")
        self.dx = None
        self.dy = None
        self.goal_yaw = None
        self.max_linear = 0.3 
        self.max_ang = 0.6
        self.dist_tolerance = 0.1
        self.ang_tolerance = 0.3
        self.goal_reached = False

    def odom_callback(self, msg):
        """Extrae x, y y yaw desde odometría"""
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.current_pose.theta = yaw

    def target_callback(self, msg:PoseStamped):
        """Almacena el objetivo recibido"""
        
        self.target_pose = Pose2D() 
        self.target_pose.x = msg.pose.position.x
        self.target_pose.y = msg.pose.position.y
        q = msg.pose.orientation
        theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.target_pose.theta = theta
        rospy.loginfo(f"New target: x={self.target_pose.x:.2f}, y={self.target_pose.x:.2f}, θ={math.degrees(self.target_pose.theta):.1f}°")

    def quaternion_to_yaw(self, x, y, z, w):
        """Convierte un cuaternión a yaw"""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def move_to_goal(self):
        """Decide qué estrategia usar"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.target_pose is None:
                rate.sleep()
                continue

            if self.approach == 1:
                self.approach_one()
            elif self.approach == 2:
                self.approach_two()
            else:
                rospy.logwarn("Invalid approach selected. Use 1 or 2.")

            rate.sleep()

    def approach_one(self):
        """Girar hacia el objetivo y luego avanzar recto"""
        twist = Twist()
        target_x, target_y, target_yaw = self.target_pose.x,self.target_pose.y,self.target_pose.theta
        current_x, current_y, current_yaw = self.current_pose.x,self.current_pose.y,self.current_pose.theta
        
        dx = target_x - current_x
        dy = target_y - current_y
        goal_yaw_error = target_yaw - current_yaw
        
        # Error angular
        yaw_error = np.arctan2(dy, dx) - current_yaw
        
        # Umbral angular
        if abs(yaw_error) > 0.2:
            twist.angular.z = 0.3 * yaw_error
            if abs(twist.angular.z) > self.max_ang:
                twist.angular.z = np.sign(twist.angular.z)*self.max_ang
        else:
            # Una vez alineado, avanzar
            dist = np.sqrt(dx**2+dy**2)
            if dist > 0.2:
                twist.linear.x = 0.3
            elif abs(goal_yaw_error) > 0.2:
                twist.angular.z = 0.3 * goal_yaw_error
                if abs(twist.angular.z) > self.max_ang:
                    twist.angular.z = np.sign(twist.angular.z)*self.max_ang
            else:
                twist.angular.z=0.0
                twist.linear.x=0.0

        self.cmd_pub.publish(twist)
    def angle_normalizer(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def approach_two(self):
        """Control continuo con feedback"""
        twist = Twist()

        target_x, target_y, target_yaw = self.target_pose.x,self.target_pose.y,self.target_pose.theta
        current_x, current_y, current_yaw = self.current_pose.x,self.current_pose.y,self.current_pose.theta
        
        dx = target_x - current_x
        dy = target_y - current_y
        dist_error = np.sqrt(dx**2+dy**2)
            
        goal_yaw_error = self.angle_normalizer(target_yaw - current_yaw)
        heading_error = self.angle_normalizer(np.arctan2(dy, dx) - current_yaw) 
        # Ganancias de control (ajustables)
        K_lin = 0.1
        K_ang = 1
        K_head = 2.5
        if dist_error < self.dist_tolerance and abs(goal_yaw_error) < self.ang_tolerance:
            if not self.goal_reached:
                rospy.loginfo("Goal reached! Stopping the robot")
                self.goal_reached = True
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else: 
            self.goal_reached = False
            twist.angular.z = K_ang*goal_yaw_error + K_head*heading_error
            if abs(twist.angular.z) > self.max_ang:
                twist.angular.z = np.sign(twist.angular.z)*self.max_ang
            twist.linear.x = K_lin*dist_error
            if twist.linear.x > self.max_linear:
                twist.linear.x = self.max_linear
        # # Control proporcional
        # if dist > 0.2:
            # twist.linear.x = K_lin * dist
            # twist.angular.z = K_ang * yaw_error
        # else:
            # # Cuando está cerca del objetivo, ajustar orientación final
            # if abs(goal_yaw_error) > 0.1:
                # twist.angular.z = K_ang * goal_yaw_error
            # else:
                # twist.linear.x = 0.0
                # twist.angular.z = 0.0
                # rospy.loginfo("Goal reached!")

        self.cmd_pub.publish(twist)

if __name__ == "__main__":
    node = MoveRobotNode()
    try:
        node.move_to_goal()
    except rospy.ROSInterruptException:
        pass


