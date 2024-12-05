#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pynput.keyboard import Key, Listener
import sys
import signal

# 初始化ROS节点
rospy.init_node('keyboard_driver', anonymous=True)

# 创建速度和布尔值的发布者
cmd_vel = rospy.Publisher('/keyboard_vel', Twist, queue_size=1)
keyboard_pub = rospy.Publisher('/keyboard', Bool, queue_size=1)

def on_press(key):
    move = Twist()
    key_pressed = Bool()

    if key == Key.up:
        move.linear.x = 0.3
    elif key == Key.down:
        move.linear.x = -0.3
    elif key == Key.left:
        move.linear.y = 0.3
    elif key == Key.right:
        move.linear.y = -0.3
    elif key == Key.esc:
        # Stop listener
        return False

    try:
        if key.char == 'q':
            move.angular.z = 0.5
        elif key.char == 'e':
            move.angular.z = -0.5
        if key.char == 'i':
            key_pressed.data = True
            keyboard_pub.publish(key_pressed)
        elif key.char == 'o':
            key_pressed.data = False
            keyboard_pub.publish(key_pressed)
    except AttributeError:
        pass

    print(move)
    cmd_vel.publish(move)

def on_release(key):
    move = Twist()
    cmd_vel.publish(move)
    if key == Key.esc:
        return False

def exit_gracefully(signum, frame):
    # Handle any cleanup here
    print("Program exited gracefully")
    sys.exit(0)

def main():
    # Set the signal handler
    signal.signal(signal.SIGINT, exit_gracefully)
    
    with Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

if __name__ == "__main__":
    main()