#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class TeleopKeyboardNode:
    def __init__(self):
        rospy.init_node('teleop_keyboard_node', anonymous=True)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.loginfo('Teleop node started. Use W/S to move forward/backward, A/D to turn, X to stop.')
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.run()

    def get_key(self):
        """Lee una tecla de forma no bloqueante."""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    def run(self):
        linear_speed = 1.0
        angular_speed = 1.0

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
            elif key == '\x03':  # Ctrl+C
                break

            self.publisher.publish(twist)

if __name__ == '__main__':
    try:
        TeleopKeyboardNode()
    except rospy.ROSInterruptException:
        pass
