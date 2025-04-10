#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from sensor_msgs.msg import Image
from my_cpp_py_pkg.msg import SimulationState  # Custom message

class StateBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_state_bridge_node')

        # Variabili per memorizzare i dati più recenti
        self.gripper_command = Float64()
        self.obj_poses = []  # Array dinamico per le pose dei cubi
        self.obj_velocities = []  # Array dinamico per le velocità dei cubi
        self.tcp_pose = PoseStamped()
        self.tcp_velocity = TwistStamped()
        self.tcp_force_torque = WrenchStamped()
        self.camera_images = []  # Array dinamico per le immagini delle telecamere

        # Parametri per i topic
        self.declare_parameter('obj_pose_topics', [""])
        self.declare_parameter('obj_velocity_topics', [""])
        self.declare_parameter('camera_topics', [""])

        # Sub to the variable topics of objs 
        obj_pose_topics = self.get_parameter('obj_pose_topics').get_parameter_value().string_array_value
        obj_velocity_topics = self.get_parameter('obj_velocity_topics').get_parameter_value().string_array_value

        self.obj_pose_subscriptions = []
        self.obj_velocity_subscriptions = []

        for topic in obj_pose_topics:
            self.obj_poses.append(PoseStamped())
            self.obj_pose_subscriptions.append(
                self.create_subscription(PoseStamped, topic, self.create_obj_pose_callback(len(self.obj_poses) - 1), 10)
            )

        for topic in obj_velocity_topics:
            self.obj_velocities.append(TwistStamped())
            self.obj_velocity_subscriptions.append(
                self.create_subscription(TwistStamped, topic, self.create_obj_velocity_callback(len(self.obj_velocities) - 1), 10)
            )

        # Sub to the variable topics of cameras 
        camera_topics = self.get_parameter('camera_topics').get_parameter_value().string_array_value
        self.camera_subscriptions = []

        for topic in camera_topics:
            self.camera_images.append(Image())
            self.camera_subscriptions.append(
                self.create_subscription(Image, topic, self.create_camera_callback(len(self.camera_images) - 1), 10)
            )

        # Sottoscrizioni ai topic fissi
        self.create_subscription(Float64, '/gripper_command', self.gripper_command_callback, 10)
        self.create_subscription(PoseStamped, '/mujoco_ros/tcp_pose', self.tcp_pose_callback, 10)
        self.create_subscription(TwistStamped, '/mujoco_ros/tcp_velocity', self.tcp_velocity_callback, 10)
        self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster_fake/wrench', self.tcp_force_torque_callback, 10)

        # Publisher per il messaggio custom Simulation_State
        self.state_publisher = self.create_publisher(SimulationState, 'mujoco_state', 10)

        # Timer per pubblicare periodicamente
        self.timer = self.create_timer(0.1, self.publish_simulation_state)

        self.get_logger().info('State Bridge Node started.')

    # Dynamic Callbacks for objects
    def create_obj_pose_callback(self, index):
        def callback(msg):
            self.obj_poses[index] = msg
        return callback

    def create_obj_velocity_callback(self, index):
        def callback(msg):
            self.obj_velocities[index] = msg
        return callback
    
    # Dynamic Callbacks for cameras
    def create_camera_callback(self, index):
        def callback(msg):
            self.camera_images[index] = msg
        return callback

    # Fixed Callbacks
    def gripper_command_callback(self, msg):
        self.gripper_command = msg

    def tcp_pose_callback(self, msg):
        self.tcp_pose = msg

    def tcp_velocity_callback(self, msg):
        self.tcp_velocity = msg

    def tcp_force_torque_callback(self, msg):
        self.tcp_force_torque = msg

    def publish_simulation_state(self):
        state_msg = SimulationState()
        state_msg.gripper_command = self.gripper_command
        state_msg.obj_poses = self.obj_poses
        state_msg.obj_velocities = self.obj_velocities
        state_msg.tcp_pose = self.tcp_pose
        state_msg.tcp_velocity = self.tcp_velocity
        state_msg.tcp_force_torque = self.tcp_force_torque
        state_msg.camera_images = self.camera_images

        self.state_publisher.publish(state_msg)


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