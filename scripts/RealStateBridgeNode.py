#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from sensor_msgs.msg import Image
from hil_serl_hiro_utils.msg import RealState  # Custom message
from std_msgs.msg import Float32MultiArray

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

##########
import numpy as np
import gymnasium as gym
from std_srvs.srv import Trigger 
from ur_msgs.msg import IOStates
import ur_msgs.srv
import threading
import time
##########

##################################################################
import tf_transformations
import random

# def quat_to_euler(q):
    # return tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

def euler_to_quat(roll, pitch, yaw):
    q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    return q  # [x, y, z, w]
##################################################################

def print_green(x):
    return print("\033[92m {}\033[00m".format(x))
 
def print_orange(x):
    return print("\033[93m {}\033[00m".format(x))
 
def print_blu(x):
    return print("\033[94m {}\033[00m".format(x))

class RealStateBridgeNode(Node):
    def __init__(self):
        super().__init__('real_state_bridge_node')


        ############################ Real UR Hard Coded safety Positions ###################################
        ##  START position:
        ## x: -0.5062012016721099
        ## y: 0.17
        ## z: 0.43

        ## MY HIGH LIMIT
        ## x: -0.43935539849167454 --> aumento max a -0.35
        ## Y = 0.3603177944808122 # aumento ma poco --> 0.41 perchè c'è piano sicurezza stra vicino lì
        ## Z: 0.584233792408213


        ## MY LOW LIMIT
        ## x: -0.6606584651129386 --> aumento a -0.77 (direzione avanti da robot verso copmuter)
        ##Y = -0.2600768840612451
        ##    z: 0.21108867841114953

        ## TEMP_REAL_POSITION = np.array([-0.5, 0.17, 0.43])
        ## POSE_LIMIT_HIGH =  TEMP_REAL_POSITION + np.array([0.1, 0.2, 0.22])  #### questi valori buoni e abbastanza larghi
        ## POSE_LIMIT_LOW = TEMP_REAL_POSITION - np.array([0.2, 0.35, 0.22])  #### questi valori buoni e abbastanza larghi
         

        ########################### AVVITATORE Hard Coded safety Positions ###################################
        ##  START position:
        ## x: -0.5031390929532893
        ## y: 0.14744824937867473
        ## z: 0.36

        ## MY HIGH LIMIT
        ## x: -0.48 --> -0.475
        ## Y = 0.164 # aumento ma poco --> 0.41 perchè c'è piano sicurezza stra vicino lì
        ## Z: 0.45


        ## MY LOW LIMIT
        ## x: -0.53
        ## Y = 0.13
        ##    z: 0.2799999 # essendo vite circa 0.32 --> TODO provare con joystick dove arriva e con quale velocità


        ## z: z: 0.32363064023408317 --> H vite circa, un po meno in realtà tipo 0,30840358685060526

        TEMP_REAL_POSITION = np.array([-0.5031390929532893, 0.14744824937867473, 0.36])
        POSE_LIMIT_HIGH =  TEMP_REAL_POSITION + np.array([0.035, 0.035, 0.09]) # leggermente più ampio ma cappo azioni lato wrapper, per avere più libertà MA movimenti più precisi 
        POSE_LIMIT_LOW = TEMP_REAL_POSITION - np.array([0.035, 0.035, 0.0701])
         
        # safety boundary box
        self.xyz_bounding_box = gym.spaces.Box(
            POSE_LIMIT_LOW[:3],
            POSE_LIMIT_HIGH[:3],
            dtype=np.float64,
        )

        # Variabili per memorizzare i dati più recenti
        self.gripper_command = Float64()
        self.gripper_act_state = Float64()
        self.previous_gripper_command = Float64() 
        self.previous_gripper_command.data = -1.0 

        self.tcp_pose = PoseStamped()
        # self.tcp_velocity = TwistStamped()
        self.tcp_force_torque = WrenchStamped()

        self.robot_action = Float32MultiArray()  # Robot action given by Gym

        # Configurazione QoS per il publisher e il subscriber di robot action, per evitare code lunghe non-gestite ma prendere sempre ultimo msg (in teoria)
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,  # Garantisce la consegna dei messaggi
        durability=DurabilityPolicy.VOLATILE,   # I messaggi non vengono conservati
        depth=1                                 # Mantiene solo l'ultimo messaggio nella coda
)
        


        # --- Robot Topics Subscribers
        self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.gripper_actuator_callback, qos_profile)
        self.create_subscription(PoseStamped, '/admittance_controller/w_T_ee', self.tcp_pose_callback, qos_profile)
        # self.create_subscription(TwistStamped, '/mujoco_ros/tcp_velocity', self.tcp_velocity_callback, 10)
        self.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.tcp_force_torque_callback, 10)

        #  --- Gym Action Subscriber 
        self.create_subscription(Float32MultiArray, 'gym_ros/robot_action', self.robot_action_callback, qos_profile)

        # -- State Publisher 
        self.state_publisher = self.create_publisher(RealState, 'real_state', 10) 

        # -- Robot's Controller Goal Publisher
        self.tcp_goal_publisher = self.create_publisher(PoseStamped, 'admittance_controller/target_pose', qos_profile)

        # - I/O Real Gripper Service Client 
        self.real_gripper_client = self.create_client(ur_msgs.srv.SetIO, '/io_and_status_controller/set_io')

        # - Real Reset Service Server 
        self.srv = self.create_service(Trigger,'real_reset',self.reset_callback)

        # Timer per pubblicare periodicamente
        self.timer = self.create_timer(0.1, self.publish_simulation_state)
        self.get_logger().info('State Bridge Node started.')

    def call_set_io_service(self, pin, state):
        # nuova versione con multithread non bloccante!
        request = ur_msgs.srv.SetIO.Request()
        request.fun = 1
        request.pin = pin
        request.state = state

        future = self.real_gripper_client.call_async(request)
        future.add_done_callback(self.io_response_callback)

    def reset_callback(self, request, response):
        
        reset_pose = PoseStamped()
        reset_pose.header.stamp = self.get_clock().now().to_msg()
        reset_pose.header.frame_id = 'base_link'

        #  My Custom Gripper Case !! (FLANGIA GIRATA 90° RISPETTO ALTRO)

        # reset_pose.pose.position.x =  -0.5 #-0.69139
        # reset_pose.pose.position.y = 0.17
        # reset_pose.pose.position.z = 0.43

        # reset_pose.pose.orientation.x  =  0.7216245107730183
        # reset_pose.pose.orientation.y  = 0.6922019755939712
        # reset_pose.pose.orientation.z  =  -0.0007271693071297483
        # reset_pose.pose.orientation.w  =  -0.010675282675617136

        #  Atlas screwdriver Case
        
        # Range di randomizzazione
        delta = 0.025

        reset_pose.pose.position.x =  -0.5031390929532893 + random.uniform(-delta, delta) 
        reset_pose.pose.position.y = 0.14744824937867473 + random.uniform(-delta, delta)
        reset_pose.pose.position.z = 0.36  #slightly above obj 

        reset_pose.pose.orientation.x  = -0.5061135480338557
        reset_pose.pose.orientation.y  = -0.4931089242003056
        reset_pose.pose.orientation.z  =  0.49645543168566075
        reset_pose.pose.orientation.w  =  0.5042069711144461

        self.tcp_goal_publisher.publish(reset_pose)
        self.get_logger().info("Reset service called, publishing reset pose.")

        ## reset gripper I/O 
        print_green("resetting gripper condtion to -> OPEN")
        self.call_set_io_service(7, 0.0) ## elettreovalvola --> solo output 7
        # self.call_set_io_service(6, 1.0)

        response.success = True
        response.message = "Reset successful"
        return response
    
    def io_response_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"[SetIO] Success: {result}")
        except Exception as e:
            self.get_logger().error(f"[SetIO] Failed: {e}")
    
    def gripper_actuator_callback(self, msg):
        closed = 239.0
        open =  0.0

        for d_state in msg.digital_out_states:
            if d_state.pin ==7:
                pin_7_state = d_state.state

        if pin_7_state:
            self.gripper_act_state.data = closed 
        else: # 6 == 7 -> Do nothing, keep last state (Open or Closed)
            self.gripper_act_state.data = open 

        self.get_logger().info(f"Gripper actuator state: {self.gripper_act_state}")

        print_orange(f"current quat X = {self.tcp_pose.pose.orientation.x},\nY =  {self.tcp_pose.pose.orientation.y}\nZ = {self.tcp_pose.pose.orientation.z}\nW = {self.tcp_pose.pose.orientation.w}")
        # 1. Quaternione → Euler
        curr_roll, curr_pitch, curr_yaw = tf_transformations.euler_from_quaternion([
            self.tcp_pose.pose.orientation.x, self.tcp_pose.pose.orientation.y, self.tcp_pose.pose.orientation.z, self.tcp_pose.pose.orientation.w
        ])
        print_green(f"ROLL = {curr_roll},\nPITCH =  {curr_pitch}\nYAW = {curr_yaw}")


        ################################## MINIMI #################################
        # ROLL = -2.9
        # PITCH =  -0.17071212282544174 --> -0.15
        # YAW = 1,47
        ############################################################
        
        ################################## MAXXIMI #################################
        # ROLL = + 2.9
        # PITCH = 0.15
        # YAW = 1,59
        ############################################################


        ######### caso avvitatore: iniziale RPY
        #  ROLL = -1.57
        #  PITCH =  0.00
        #  YAW = 1.57

        ###### posizione inserimento bit nella vite (con PC messo sugli scotch) circa:
        # x: -0.5052123373348968
        # y: 0.14769605069215744
        # z: 0.30840358685060526


    def tcp_pose_callback(self, msg):
        self.tcp_pose = msg

    # def tcp_velocity_callback(self, msg):
        # self.tcp_velocity = msg

    def tcp_force_torque_callback(self, msg):
        self.tcp_force_torque = msg


    # Function to clip robot actions within safety boundaries
    def clip_safety_box(self, pose: np.ndarray) -> np.ndarray:
        """Clip the pose to be within the safety box."""
        pose[:3] = np.clip(
            pose[:3], self.xyz_bounding_box.low, self.xyz_bounding_box.high
        ) 
        return pose

    def robot_action_callback(self, msg):

        self.robot_action = msg # [X, Y, Z, Gripper_command]
        # self.get_logger().info(f"Azione ricevuta: {msg.data}") 

        translation = Float32MultiArray()
        translation.data = msg.data[:3]  # first 3 elements

        new_action_pose = PoseStamped()
        new_action_pose.header.stamp = self.get_clock().now().to_msg()
        new_action_pose.header.frame_id = 'base_link'

        new_action_pose.pose.position.x = self.tcp_pose.pose.position.x + translation.data[0]
        new_action_pose.pose.position.y = self.tcp_pose.pose.position.y + translation.data[1]
        new_action_pose.pose.position.z = self.tcp_pose.pose.position.z + translation.data[2]
        new_action_pose.pose.orientation = self.tcp_pose.pose.orientation

        print_orange(f"\n  goal originale: {new_action_pose.pose.position.x}, {new_action_pose.pose.position.y}, {new_action_pose.pose.position.z}")
 
        # Clip robot action 
        clipped_position = self.clip_safety_box(
            np.array([
                new_action_pose.pose.position.x,
                new_action_pose.pose.position.y,
                new_action_pose.pose.position.z
            ])
        )
        
        new_action_pose.pose.position.x = clipped_position[0]
        new_action_pose.pose.position.y = clipped_position[1]
        new_action_pose.pose.position.z = clipped_position[2]
        # print_green(f" posa goal CLIPPATA: {new_action_pose.pose.position.x}, {new_action_pose.pose.position.y}, {new_action_pose.pose.position.z}")
 
        self.tcp_goal_publisher.publish(new_action_pose)

        
        # ----------------------------------------- GRIPPER -----------------------------------------
        # -------------------------------------------------------------------------------------------

        # fourth = Gripper
        self.gripper_command.data =  msg.data[3] 
 
        # Check for new command
        if self.gripper_command.data != self.previous_gripper_command.data:
            self.previous_gripper_command.data = self.gripper_command.data

            if self.gripper_command.data == 0:
                # Open the Gripper
                self.call_set_io_service(7, 0.0) 
                # self.call_set_io_service(6, 1.0) # elettrovalvola = solo output 7
            else: # ==239
                # Close the Gripper
                # self.call_set_io_service(6, 0.0) # elettrovalvola = solo output 7
                self.call_set_io_service(7, 1.0)
        else:
            # Unchanged Command, Ignore
            pass

        # -------------------------------------------------------------------------------------------
        # -------------------------------------------------------------------------------------------

    def publish_simulation_state(self):
        
        state_msg = RealState() 
        state_msg.gripper_state = self.gripper_act_state
        state_msg.tcp_pose = self.tcp_pose
        # state_msg.tcp_velocity = self.tcp_velocity
        state_msg.tcp_force_torque = self.tcp_force_torque

        self.state_publisher.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealStateBridgeNode()
    
    # Async Thread to avoid Race Conditions and Deadlocks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # Execute on separate Thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        rclpy.spin(node)
        time.sleep(0.1) 
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()