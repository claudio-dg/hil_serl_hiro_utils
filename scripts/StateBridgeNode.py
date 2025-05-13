#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from sensor_msgs.msg import Image
from my_cpp_py_pkg.msg import SimulationState  # Custom message
from std_msgs.msg import Float32MultiArray

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class StateBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_state_bridge_node')

        # Variabili per memorizzare i dati più recenti
        self.gripper_command = Float64()
        self.gripper_act_state = Float64()
        ######
        self.previous_gripper_command = Float64()  # Nuova variabile per memorizzare il comando precedente
        self.previous_gripper_command.data = -1.0  # Inizializza con un valore impossibile (ad esempio, -1)
        ######

        self.obj_poses = []  # Array dinamico per le pose dei cubi
        self.obj_velocities = []  # Array dinamico per le velocità dei cubi
        self.tcp_pose = PoseStamped()
        self.tcp_velocity = TwistStamped()
        self.tcp_force_torque = WrenchStamped()
        self.camera_images = []  # Array dinamico per le immagini delle telecamere

        self.robot_action = Float32MultiArray()  # Variabile per memorizzare l'azione ricevuta lato gym

        # Parametri per i topic
        self.declare_parameter('states.obj_pose_topics', [""])
        self.declare_parameter('states.obj_velocity_topics', [""])
        self.declare_parameter('states.camera_topics', [""])

        self.declare_parameter('actions.tcp_pose_topics', [""])
        self.declare_parameter('actions.gripper_cmd_topics', [""])

        # Sub to the variable topics of objs 
        obj_pose_topics = self.get_parameter('states.obj_pose_topics').get_parameter_value().string_array_value
        obj_velocity_topics = self.get_parameter('states.obj_velocity_topics').get_parameter_value().string_array_value

        action_tcp_pose_topics = self.get_parameter('actions.tcp_pose_topics').get_parameter_value().string_array_value
        action_gripper_cmd_topics = self.get_parameter('actions.gripper_cmd_topics').get_parameter_value().string_array_value

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
        camera_topics = self.get_parameter('states.camera_topics').get_parameter_value().string_array_value
        self.camera_subscriptions = []

        for topic in camera_topics:
            self.camera_images.append(Image())
            self.camera_subscriptions.append(
                self.create_subscription(Image, topic, self.create_camera_callback(len(self.camera_images) - 1), 10)
            )

        # Pub to the variable action topics from param file
        self.tcp_pose_publishers = []
        self.gripper_cmd_publishers = []

        # Configurazione QoS per il publisher e il subscriber di robot action, per evitare code lunghe non-gestite ma prendere sempre ultimo msg (in teoria)
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,  # Garantisce la consegna dei messaggi
        durability=DurabilityPolicy.VOLATILE,   # I messaggi non vengono conservati
        depth=1                                 # Mantiene solo l'ultimo messaggio nella coda
)

        for topic in action_tcp_pose_topics:
            self.tcp_pose_publishers.append(
                self.create_publisher(PoseStamped, topic, qos_profile) 
            )

        
        for topic in action_gripper_cmd_topics:
            self.gripper_cmd_publishers.append(
                self.create_publisher(Float64, topic, qos_profile)
            )

        # Sottoscrizioni ai topic fissi
        self.create_subscription(Float64, '/mujoco_ros/gripper_actuator_state', self.gripper_actuator_callback, 10)
        self.create_subscription(PoseStamped, '/mujoco_ros/tcp_pose', self.tcp_pose_callback, 10)
        self.create_subscription(TwistStamped, '/mujoco_ros/tcp_velocity', self.tcp_velocity_callback, 10)
        self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster_fake/wrench', self.tcp_force_torque_callback, 10)
        
        #  Sottoscrizione all'azione del robot pubblicata da gym
        self.create_subscription(Float32MultiArray, 'gym_ros/robot_action', self.robot_action_callback, qos_profile)

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
    
    def gripper_actuator_callback(self, msg):
        self.gripper_act_state = msg

    def tcp_pose_callback(self, msg):
        self.tcp_pose = msg

    def tcp_velocity_callback(self, msg):
        self.tcp_velocity = msg

    def tcp_force_torque_callback(self, msg):
        self.tcp_force_torque = msg

    # Callback per il topic "gym_ros/robot_action"
    def robot_action_callback(self, msg):
        self.robot_action = msg
        # self.get_logger().info(f"Azione ricevuta: {msg.data}") 
        #  ( primi tre valori = traslazione asse x,y,z, quarto valore = ccontrollo del gripper)

        translation = Float32MultiArray()
        translation.data = msg.data[:3]  # Prendi i primi tre valori

        new_action_pose = PoseStamped()
        new_action_pose.header.stamp = self.get_clock().now().to_msg()
        new_action_pose.header.frame_id = 'base_link'

        new_action_pose.pose.position.x = self.tcp_pose.pose.position.x + translation.data[0]
        new_action_pose.pose.position.y = self.tcp_pose.pose.position.y + translation.data[1]
        new_action_pose.pose.position.z = self.tcp_pose.pose.position.z + translation.data[2]
        new_action_pose.pose.orientation = self.tcp_pose.pose.orientation

        # self.publisher.publish(new_pose)
        # self.get_logger().info(f" *** p Basletta: x={self.tcp_pose.pose.position.x}, y={self.tcp_pose.pose.position.y}, z={self.tcp_pose.pose.position.z}")
        # self.get_logger().info(f" ### pos pub: x={new_action_pose.pose.position.x}, y={new_action_pose.pose.position.y}, z={new_action_pose.pose.position.z}")
        # self.get_logger().info(f" +++ traslazione: x={translation.data[0]}, y={translation.data[1]}, z={translation.data[2]}")
        # self.get_logger().info(f"\n")

        self.tcp_pose_publishers[0].publish(new_action_pose)

        
        # Quarto valore: comando del gripper
        self.gripper_command.data =  msg.data[3] 
 
        # Confronta il comando ricevuto con il comando precedente
        if self.gripper_command.data != self.previous_gripper_command.data:
            # Aggiorna il comando precedente
            self.previous_gripper_command.data = self.gripper_command.data

            # Pubblica il nuovo comando
            self.get_logger().info(f"gripper ricevuto da BRIDGE!!  =  {self.gripper_command.data}")
            self.gripper_cmd_publishers[0].publish(self.gripper_command)
        else:
            # Comando invariato, non pubblicare
            pass

        
    def publish_simulation_state(self):
        
        state_msg = SimulationState()
        state_msg.gripper_command = self.gripper_act_state
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