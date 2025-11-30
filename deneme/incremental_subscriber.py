#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Float32, Bool#for message types


def main(args=None):
    rclpy.init(args=args)#this is needed to create a node
    node = rclpy.create_node('incremental_subscriber')#name of the node

    error_pub = node.create_publisher(Bool, 'incremental_error', 10)

    def callback(msg):
        value = msg.data
        node.get_logger().info(f'incremental_data: {value}')

        err_msg = Bool()
        err_msg.data = (value > 70.0)
        error_pub.publish(err_msg)

        if err_msg.data:
            node.get_logger().warn('incremental_error TRUE (value > 70)')

    node.create_subscription(Float32, 'incremental_data', callback, 10)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
