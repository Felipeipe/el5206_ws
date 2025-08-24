#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pandas as pd
import sys
import tty
import termios

class TeleopKeyboardNode:
    def __init__(self):
        rospy.init_node('teleop_keyboard_node', anonymous=True)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/ground_truth/state', Odometry, self.gt_callback)

        rospy.loginfo('Teleop node started. Use W/S to move forward/backward, A/D to turn, X to stop.')
        self.ox = 0.0 
        self.oy = 0.0
        self.oyaw = 0.0
        self.gtx = 0.0
        self.gty = 0.0
        self.gtyaw = 0.0

        self.settings = termios.tcgetattr(sys.stdin)
        self.run()
    def quaternion_to_yaw(self, x, y, z, w):
        import math
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def gt_callback(self, msg:Odometry):
        self.gtx = msg.pose.pose.position.x
        self.gty = msg.pose.pose.position.y
        gtq = msg.pose.pose.orientation
        self.gtyaw = self.quaternion_to_yaw(gtq.x, gtq.y, gtq.z, gtq.w)

    def odom_callback(self, msg: Odometry):
        self.ox = msg.pose.pose.position.x
        self.oy = msg.pose.pose.position.y
        oq = msg.pose.pose.orientation
        self.oyaw = self.quaternion_to_yaw(oq.x, oq.y, oq.z, oq.w)

    def get_key(self):
        """Lee una tecla de forma no bloqueante."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    def run(self):
        linear_speed = 0.3
        angular_speed = 0.3
        df = pd.DataFrame(columns=['ox', 'oy', 'oyaw', 'gtx', 'gty', 'gtyaw'])

        while not rospy.is_shutdown():
            key = self.get_key()
            twist = Twist()

            if key == 'w':
                twist.linear.x = linear_speed
            elif key == 'a':
                twist.angular.z = angular_speed
            elif key == 's':
                twist.linear.x = -linear_speed/2
            elif key == 'd':
                twist.angular.z = -angular_speed
            elif key == 'x':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                data = {
                    'ox':self.ox,
                    'oy':self.oy,
                    'oyaw':self.oyaw,
                    'gtx':self.gtx,
                    'gty':self.gty,
                    'gtyaw':self.gtyaw
                }
                row = pd.DataFrame([data])
                df = pd.concat([df, row], ignore_index=True)
                df.to_csv('tabla_sync.csv', index=False)
            elif key == '\x03':  # Ctrl+C
                break

            self.publisher.publish(twist)

if __name__ == '__main__':
    try:
        TeleopKeyboardNode()
    except rospy.ROSInterruptException:
        pass
