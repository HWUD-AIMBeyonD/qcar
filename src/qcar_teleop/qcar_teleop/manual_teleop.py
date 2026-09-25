#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Settings
MAX_LINEAR_SPEED = 0.3  # m/s - low speed for safety
MAX_ANGULAR_SPEED = 1.0  # rad/s

msg = """
---------------------------
QCar Manual Control
---------------------------
MOVING:
   w
 a   d
   s

REVERSE STEERING:
 q       e
  \\     /
   reverse

x : Force Stop
z : Quit

CTRL-C to quit
"""

class ManualTeleop(Node):
    def __init__(self):
        super().__init__('manual_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Manual Teleop Node Started. Use WASD to drive.")

    def publish_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(angular)
        self.publisher_.publish(twist)

def get_key(settings):
    # Low-level system call to get a single character from stdin
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = ManualTeleop()

    settings = termios.tcgetattr(sys.stdin)
    linear_speed = 0.0
    angular_speed = 0.0

    try:
        print(msg)
        print(f"Max speed: {MAX_LINEAR_SPEED} m/s")

        while rclpy.ok():
            key = get_key(settings)

            if key == 'w':
                linear_speed = MAX_LINEAR_SPEED
                angular_speed = 0.0
            elif key == 's':
                linear_speed = -MAX_LINEAR_SPEED
                angular_speed = 0.0
            elif key == 'a':
                linear_speed = MAX_LINEAR_SPEED * 0.5
                angular_speed = MAX_ANGULAR_SPEED
            elif key == 'd':
                linear_speed = MAX_LINEAR_SPEED * 0.5
                angular_speed = -MAX_ANGULAR_SPEED
            elif key == 'q':
                linear_speed = -MAX_LINEAR_SPEED * 0.5
                angular_speed = MAX_ANGULAR_SPEED
            elif key == 'e':
                linear_speed = -MAX_LINEAR_SPEED * 0.5
                angular_speed = -MAX_ANGULAR_SPEED
            elif key == 'x':
                linear_speed = 0.0
                angular_speed = 0.0
            elif key == 'z' or key == '\x03':
                break
            else:
                linear_speed = 0.0
                angular_speed = 0.0

            node.publish_twist(linear_speed, angular_speed)

    except Exception as e:
        print(e)

    finally:
        # Publish stop command before exiting
        node.publish_twist(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

