#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

def configure_terminal():
    """Set terminal to raw mode and return original settings."""
    original_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    return original_settings

def restore_terminal(settings):
    """Restore terminal settings to original state."""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def read_input():
    """Non-blocking read of a single key press."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return ''

def send_command(publisher, vx, wz):
    """Send a Twist command to the robot."""
    msg = Twist()
    msg.linear.x = vx
    msg.angular.z = wz
    publisher.publish(msg)

def keyboard_loop(publisher):
    """Main loop to handle key input and send motion commands."""
    linear_speed = 0.2
    angular_speed = 0.5
    keymap = {
        'w': (linear_speed, 0.0),
        's': (-linear_speed, 0.0),
        'a': (0.0, angular_speed),
        'd': (0.0, -angular_speed)
    }

    original_settings = configure_terminal()

    try:
        while not rospy.is_shutdown():
            key = read_input()
            if key in keymap:
                vx, wz = keymap[key]
                send_command(publisher, vx, wz)
            elif key == 'q':
                break
            else:
                send_command(publisher, 0.0, 0.0)
            rospy.sleep(0.3)
    finally:
        send_command(publisher, 0.0, 0.0)
        restore_terminal(original_settings)

if __name__ == '__main__':
    rospy.init_node('keyboard_teleop_node', anonymous=True)
    vel_publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    keyboard_loop(vel_publisher)
