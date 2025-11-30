#!/usr/bin/env python3

import threading #background thread
import sys #low-level terminal key reading
import termios #low-level terminal key reading
import tty #low-level terminal key reading
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


def getch_nonblock(callback):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        while True:
            ch = sys.stdin.read(1)#doesn't return until a key is pressed!
            callback(ch)
    except Exception:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    #this code is used for multiple reading(real-time control)


def main(args=None):
    rclpy.init(args=args)#initializes ros2 client

    node = rclpy.create_node('data_publisher')

    # Both topics now use Float32
    incremental_pub = node.create_publisher(Float32, 'incremental_data', 10)
    random_pub = node.create_publisher(Float32, 'random_data', 10)

    inc_state = {'value': 35.0}   # start from 35.0 (float)
    rand_state = {'value': 50.0}  # start from 50.0 (float)

    def on_key(ch):
        if ch == 'w':
            inc_state['value'] += 1.0
        elif ch == 's':
            inc_state['value'] -= 1.0

    kb = threading.Thread(target=getch_nonblock, args=(on_key,), daemon=True)
    kb.start()

    def publish_incremental():#definition of first node
        msg = Float32()
        msg.data = inc_state['value']
        node.get_logger().info(f'incremental_data: {msg.data}')
        incremental_pub.publish(msg)

    def publish_random():#definition of second node
        step = random.choice([5.0, -5.0])
        rand_state['value'] += step

        if rand_state['value'] > 100.0 or rand_state['value'] < 0.0:
            rand_state['value'] = 50.0

        msg = Float32()
        msg.data = rand_state['value']
        node.get_logger().info(f'random_data: {msg.data}')
        random_pub.publish(msg)

    node.create_timer(0.5, publish_incremental)
    node.create_timer(1.0, publish_random)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
