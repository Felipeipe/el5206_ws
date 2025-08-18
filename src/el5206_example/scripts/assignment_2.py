#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
import math

class MoveRobotNode:
    def __init__(self):
        rospy.init_node("move_robot_node", anonymous=True)

        self.approach = 1 # o 2

        # Publishers / Subscribers
        self.cmd_pub = rospy.Publisher("completa tópico velocidades", Twist, queue_size=10)
        rospy.Subscriber("completa tópico odometría", Odometry, self.odom_callback)
        rospy.Subscriber("completa tópico target", Pose2D, self.target_callback)

        # Variables de estado
        self.current_pose = Pose2D()
        self.target_pose = None

        rospy.loginfo(f"Node started. Using approach {self.approach}")

    def odom_callback(self, msg):
        """Extrae x, y y yaw desde odometría"""
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.current_pose.theta = yaw

    def target_callback(self, msg):
        """Almacena el objetivo recibido"""
        self.target_pose = msg
        rospy.loginfo(f"New target: x={msg.x:.2f}, y={msg.y:.2f}, θ={math.degrees(msg.theta):.1f}°")

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

        dx = # target_x - current_x
        dy = # target_y - current_y
        goal_yaw = # investigar función trigonométrica útil. Pista: es atan (pero no atan normal, INVESTIGAR qué usar)

        # Error angular
        yaw_error = #goal_yaw - current_yaw
        yaw_error = math.#completar las funciones trigonométricas como antes!

        # Umbral angular
        if abs(yaw_error) > #escoger umbral angular:
            twist.angular.z = 0.3 * yaw_error
        else:
            # Una vez alineado, avanzar
            dist = #completar distancia euclidiana!
            if dist > #escoger umbral de distancia:
                twist.linear.x = 0.2

        # self.cmd_pub.publish(COMPLETAR EL MENSAJE NECESARIO)

    def approach_two(self):
        """Control continuo con feedback"""
        twist = Twist()

        dx = #Completar
        dy = #Completar
        dist = #Completar
        goal_yaw = #Completar

        yaw_error = #Completar
        yaw_error = #Completar

        #COMPLETAR ESTA FUNCIÓN CASI POR COMPLETO

if __name__ == "__main__":
    node = MoveRobotNode()
    try:
        node.move_to_goal()
    except rospy.ROSInterruptException:
        pass


