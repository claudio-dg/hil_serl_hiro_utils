#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from sensor_msgs.msg import Image
from my_cpp_py_pkg.msg import SimulationState  # Custom message

##########################################################################################################
#  parametrizzare meglio per rendere generico, ma prima capire esattamente cosa bisogna inviare e cosa no
##########################################################################################################


class StateBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_state_bridge_node')

        # Variabili per memorizzare i dati più recenti
        self.gripper_command = Float64()
        self.cube_pose = PoseStamped()
        self.cube_velocity = TwistStamped()
        self.tcp_pose = PoseStamped()
        self.tcp_velocity = TwistStamped()
        self.tcp_force_torque = WrenchStamped()
        self.right_handcam_image = Image()
        self.left_handcam_image = Image()

        # Sottoscrizioni ai topic
        self.create_subscription(Float64, '/gripper_command', self.gripper_command_callback, 10)
        self.create_subscription(PoseStamped, '/mujoco_ros/cube_pose', self.cube_pose_callback, 10)
        self.create_subscription(TwistStamped, '/mujoco_ros/cube_velocity', self.cube_velocity_callback, 10)
        self.create_subscription(PoseStamped, '/mujoco_ros/tcp_pose', self.tcp_pose_callback, 10)
        self.create_subscription(TwistStamped, '/mujoco_ros/tcp_velocity', self.tcp_velocity_callback, 10)
        self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster_fake/wrench', self.tcp_force_torque_callback, 10)
        self.create_subscription(Image, 'right_handcam/image/color', self.right_handcam_callback, 10)
        self.create_subscription(Image, 'left_handcam/image/color', self.left_handcam_callback, 10)

        # Publisher per il messaggio custom Simulation_State
        self.state_publisher = self.create_publisher(SimulationState, 'mujoco_state', 10)

        # Timer to periodically publish
        self.timer = self.create_timer(0.1, self.publish_simulation_state)

        self.get_logger().info('State Bridge Node started.')

    # Callbacks to update variables with Mujoco's read data 
    def gripper_command_callback(self, msg):
        self.gripper_command = msg

    def cube_pose_callback(self, msg):
        self.cube_pose = msg

    def tcp_pose_callback(self, msg):
        self.tcp_pose = msg

    def cube_velocity_callback(self, msg):
        self.cube_velocity = msg

    def tcp_velocity_callback(self, msg):
        self.tcp_velocity = msg

    def tcp_force_torque_callback(self, msg):
        self.tcp_force_torque = msg

    def right_handcam_callback(self, msg):
        self.right_handcam_image = msg

    def left_handcam_callback(self, msg):
        self.left_handcam_image = msg

    # publish method
    def publish_simulation_state(self):
        state_msg = SimulationState()
        state_msg.gripper_command = self.gripper_command
        state_msg.cube_pose = self.cube_pose
        state_msg.tcp_pose = self.tcp_pose
        state_msg.cube_velocity = self.cube_velocity
        state_msg.tcp_velocity = self.tcp_velocity
        state_msg.tcp_force_torque = self.tcp_force_torque
        state_msg.right_handcam_image = self.right_handcam_image
        state_msg.left_handcam_image = self.left_handcam_image

        self.state_publisher.publish(state_msg)
        # self.get_logger().info('Published simulation state.')

def main(args=None):
    rclpy.init(args=args)
    node = StateBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()