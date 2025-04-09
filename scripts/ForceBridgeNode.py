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
            # '/mj_utils_plugin/tcp_force_publisher',
            'mujoco_ros/tcp_force_publisher',
            self.force_callback,
            10
        )

        # Subscriber to the Vector3Stamped topic for torque
        self.torque_subscription = self.create_subscription(
            Vector3Stamped,
            'mujoco_ros/tcp_torque_publisher',
            self.torque_callback,
            10
        )

        # Publisher to the UR WrenchStamped topic
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
        wrench_msg.wrench.force.x = -msg.vector.x/13
        wrench_msg.wrench.force.y = -msg.vector.y/13
        wrench_msg.wrench.force.z = -msg.vector.z/13
        # wrench_msg.wrench.force.z = msg.vector.z + add_factor
        self.i += 1

        # Cap each component of the force vector to a maximum value
        max_force = 25.0
        force_components = ['x', 'y', 'z']
        for component in force_components:
            value = float(getattr(wrench_msg.wrench.force, component))
            if value > max_force:
                setattr(wrench_msg.wrench.force, component, float(max_force))
            elif value < -max_force:
                setattr(wrench_msg.wrench.force, component, float(-max_force))

        # senza gripper versione finale diviso 5 va bene ma si può anche aumentare volendo (es diviso 6/7)
        # CON GRIPPER ESSENDO CUBO E MOLTO PIU GROSSO/COMPLESSO SERVE DIVISIONE MAGGIORE (con 13 circa sembra abbastanza decente)
        
        # Map the torque vector from the latest torque message
        wrench_msg.wrench.torque.x = -self.latest_torque.vector.x /9
        wrench_msg.wrench.torque.y = -self.latest_torque.vector.y /9
        wrench_msg.wrench.torque.z = -self.latest_torque.vector.z /9 

        # Cap each component of the torque vector to a maximum value
        max_torque = 15.0
        force_components = ['x', 'y', 'z']
        for component in force_components:
            value = float(getattr(wrench_msg.wrench.torque, component))
            if value > max_torque:
                setattr(wrench_msg.wrench.torque, component, float(max_torque))
            elif value < -max_torque:
                setattr(wrench_msg.wrench.torque, component, float(-max_torque))

        # if wrench_msg.wrench.force.z > 10 or wrench_msg.wrench.force.z < -10:
            # self.get_logger().error(f"########### FORZA Z pubblicata : {wrench_msg.wrench.force.z}")
            # self.get_logger().warn(f"########### letto da plugin muj : {msg.vector.z}")
            # rclpy.shutdown()
        if wrench_msg.wrench.torque.x > 5 or wrench_msg.wrench.torque.x < -5:
            self.get_logger().warn(f"########### TORQUE X PUB : {wrench_msg.wrench.torque.x}")
               
        # Publish the converted message
        self.publisher.publish(wrench_msg)

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