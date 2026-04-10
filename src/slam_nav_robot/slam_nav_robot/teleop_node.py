#!/usr/bin/env python3
"""
Keyboard teleoperation node for the SLAM navigation robot.
Controls:
  w / s  — increase / decrease linear speed
  a / d  — turn left / right
  space  — full stop
  q      — quit
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# Terminal key codes
KEY_W = 'w'
KEY_S = 's'
KEY_A = 'a'
KEY_D = 'd'
KEY_SPACE = ' '
KEY_Q = 'q'

BANNER = """
┌─────────────────────────────────┐
│   SLAM Bot Keyboard Teleop      │
│                                 │
│   w — forward                   │
│   s — backward                  │
│   a — turn left                 │
│   d — turn right                │
│   SPACE — stop                  │
│   q — quit                      │
└─────────────────────────────────┘
"""

# Speed settings
LINEAR_STEP = 0.05   # m/s per keypress
ANGULAR_STEP = 0.1    # rad/s per keypress
MAX_LINEAR = 0.3    # m/s
MAX_ANGULAR = 1.5    # rad/s


def get_key(settings):
    """Read a single keypress from stdin (non-blocking)."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear = 0.0
        self.angular = 0.0
        self.get_logger().info('Teleop node started. Press q to quit.')

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher.publish(msg)

    def stop(self):
        self.linear = 0.0
        self.angular = 0.0
        self.publish_velocity()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    settings = termios.tcgetattr(sys.stdin)
    print(BANNER)

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == KEY_Q:
                break
            elif key == KEY_W:
                node.linear = clamp(node.linear + LINEAR_STEP,
                                    -MAX_LINEAR, MAX_LINEAR)
                node.angular = 0.0
            elif key == KEY_S:
                node.linear = clamp(node.linear - LINEAR_STEP,
                                    -MAX_LINEAR, MAX_LINEAR)
                node.angular = 0.0
            elif key == KEY_A:
                node.angular = clamp(node.angular + ANGULAR_STEP,
                                     -MAX_ANGULAR, MAX_ANGULAR)
            elif key == KEY_D:
                node.angular = clamp(node.angular - ANGULAR_STEP,
                                     -MAX_ANGULAR, MAX_ANGULAR)
            elif key == KEY_SPACE:
                node.stop()
                print('  STOP')
                continue

            if key in (KEY_W, KEY_S, KEY_A, KEY_D):
                node.publish_velocity()
                print(f'  linear: {node.linear:+.2f} m/s  '
                      f'angular: {node.angular:+.2f} rad/s')

    except Exception as e:
        print(f'Error: {e}')
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
