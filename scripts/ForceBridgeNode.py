#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, WrenchStamped

class ForceBridgeNode(Node):
    def __init__(self):
        super().__init__('force_bridge_node')

        # Subscriber to the Vector3Stamped topic for force
        self.force_subscription = self.create_subscription(
            Vector3Stamped,
            '/mj_utils_plugin/tcp_force_publisher',
            self.force_callback,
            10
        )

        # Subscriber to the Vector3Stamped topic for torque
        self.torque_subscription = self.create_subscription(
            Vector3Stamped,
            '/mj_utils_plugin/tcp_torque_publisher',
            self.torque_callback,
            10
        )

        # Publisher to the WrenchStamped topic
        self.publisher = self.create_publisher(
            WrenchStamped,
            '/force_torque_sensor_broadcaster_fake/wrench',
            10
        )

        self.get_logger().info('Force Bridge Node started.')

        # Variables to store the latest torque values
        self.latest_torque = Vector3Stamped()

        self.i = 0

    def force_callback(self, msg: Vector3Stamped):
        # Create a WrenchStamped message
        wrench_msg = WrenchStamped()

        # Copy the header from the incoming message
        wrench_msg.header = msg.header

        wrench_msg.header.frame_id = "tool0"
        
        add_factor = 0
        if self.i > 50:
            add_factor = 20            
        if self.i > 100:
            add_factor = 0
            self.i = 0

        # Map the force vector
        wrench_msg.wrench.force.x = msg.vector.x/500
        wrench_msg.wrench.force.y = msg.vector.y/500
        wrench_msg.wrench.force.z = msg.vector.z/500#-(msg.vector.z + 9.81) 
        # wrench_msg.wrench.force.z = msg.vector.z + add_factor

        

        # Map the torque vector from the latest torque message
        wrench_msg.wrench.torque.x = self.latest_torque.vector.x 
        wrench_msg.wrench.torque.y = self.latest_torque.vector.y
        wrench_msg.wrench.torque.z = self.latest_torque.vector.z     

        # valori da AGGIUNGERE per "TARARE" presenza del gripper:
        # x: +0.008161112120385606
        # y: -10.273762241190953 # ma poi perchè lungo asse y???
        # z: -1.3435256533282351
        #  anche se senza tarare srobot sta fermo cmq, perchè? y>5 non dovrebbe muoverlo?

        self.i += 1

        # Publish the converted message
        self.publisher.publish(wrench_msg)

        if wrench_msg.wrench.force.z > 10 or wrench_msg.wrench.force.z < -10:
            self.get_logger().warn(f"########### FORZA Z pubblicata : {wrench_msg.wrench.force.z}")
            self.get_logger().error(f"########### letto da plugin muj : {msg.vector.z}")
            # rclpy.shutdown()

    def torque_callback(self, msg: Vector3Stamped):
        # Update the latest torque values
        self.latest_torque = msg

def main(args=None):
    rclpy.init(args=args)
    node = ForceBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()