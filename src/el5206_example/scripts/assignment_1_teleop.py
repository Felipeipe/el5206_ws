#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        self.publisher = self.create_publisher(Twist, 'rellena aquí el tópico de velocidades', 10)
        self.get_logger().info('Teleop node started. Use WASD to move, X to stop.')

        self.settings = termios.tcgetattr(sys.stdin)
        self.run()

    # Puedes modificar esta función para leer las teclas con lo que prefieras
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        # linear = 
        # angular = 

        while rclpy.ok():
            key = self.get_key()
            twist = Twist()

            if key == 'w':
                #Completar

            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
