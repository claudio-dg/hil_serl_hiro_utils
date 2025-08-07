#!/usr/bin/env python3
from __future__ import print_function
from dataclasses import dataclass
from enum import Enum
import inputs # attivare conda env hilserl per avere librerie
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float64

## per gripper (UR digital IO) service
import ur_msgs.srv

class ControllerType(Enum):
    XBOX = "xbox"

@dataclass
class ControllerConfig:
    resolution: dict
    scale: dict

class JoystickIntervention():
    CONTROLLER_CONFIGS = {
        ControllerType.XBOX: ControllerConfig(
            # XBOX controller joystick values have 16 bit resolution [0, 65535]
            resolution={
                'ABS_X': 2**16,
                'ABS_Y': 2**16,
                'ABS_RX': 2**16,
                'ABS_RY': 2**16,
                'ABS_Z': 2**8,
                'ABS_RZ': 2**8,
                'ABS_HAT0X': 1.0,
            },
            scale={
                'ABS_X': 0.1*9, 
                'ABS_Y': 0.1*9, 
                'ABS_RX': 0.3, ## ignored
                'ABS_RY': 0.3, ## ignored
                
                # per training policy rlpd facio in modo di avere comandi tra -0.99 e 0.99 moltiplicando per 1.65 (prima erano tra -0.6 e 0.6 per motivo scritto sopra)
                'ABS_Z': 0.015*5*1.65, 
                'ABS_RZ': 0.015*5*1.65,
                'ABS_HAT0X': 0.3,
            }
        ),
    }
    def __init__(self,  action_indices=None, controller_type=ControllerType.XBOX, service_client=None, node=None):
        
        self.service_client = service_client  # gripper service client
        self.node = node  

        self.gripper_enabled = True
        self.controller_type = controller_type
        self.controller_config = self.CONTROLLER_CONFIGS[controller_type]
        
        # Controller state
        self.x_axis = 0
        self.y_axis = 0
        self.z_axis = 0
        self.rx_axis = 0
        self.ry_axis = 0
        self.rz_axis = 0
        self.left = False   # Left bumper to close gripper
        self.right = False  # Right bumper to open gripper
        self.prev_left = False  
        self.prev_right = False 

        # Start controller reading thread
        self.running = True
        self.thread = threading.Thread(target=self._read_gamepad)
        self.thread.daemon = True
        self.thread.start()
        self.intervened = False  
        self.gripper_intervened = False  
        self.expert_a = np.zeros(6)
        self.curr_pose = None
    
    def _reset_cmds(self):
        self.x_axis = 0
        self.y_axis = 0
        self.z_axis = 0
        self.rx_axis = 0
        self.ry_axis = 0
        self.rz_axis = 0
    
    def _read_gamepad(self):
        useful_codes = ['ABS_X', 'ABS_Y', 'ABS_RX', 'ABS_RY', 'ABS_Z', 'ABS_RZ', 'ABS_HAT0X']
        
        # Store consecutive event counters and values
        event_counter = {
            'ABS_X': 0,
            'ABS_Y': 0,
            'ABS_RX': 0,
            'ABS_RY': 0,
            'ABS_Z': 0,
            'ABS_RZ': 0,
            'ABS_HAT0X': 0,
        }
    
        while self.running:
            try:
                # Get fresh events
                events = inputs.get_gamepad()
                latest_events = {}
                for event in events:
                    latest_events[event.code] = event.state
                # Process events
                for code in useful_codes:
                    if code in latest_events:
                        event_counter[code] += 1
                        current_value = latest_events[code]
                                            
                    # Only update if we've seen the same value 2 times
                    if event_counter[code] >= 1:
                        # Calculate relative changes based on the axis
                        # Normalize the joystick input values to range [-1, 1] expected by the environment
                        resolution = self.controller_config.resolution[code]

                        normalized_value = current_value / (resolution / 2)
                        scaled_value = normalized_value * self.controller_config.scale[code]

                        if code == 'ABS_Y': # su giu cursore 
                            self.x_axis =   scaled_value                            

                        elif code == 'ABS_X': # dx sx cursore
                            self.y_axis =  scaled_value

                        elif code == 'ABS_RZ': # grilleto DX (RT)
                            self.z_axis = scaled_value

                        elif code == 'ABS_Z': # grilleto SX (LT)
                            # Flip sign so this will go in the down direction
                            self.z_axis = -scaled_value

                        ### ignoro cursore destro
                        # elif code == 'ABS_RX': # cursore R3 destra sinistra
                            #self.rx_axis = scaled_value
                            #print("5  cmd -- ABS RX R3", code, self.rx_axis)

                        # elif code == 'ABS_RY': # da -0.3 a 0.3
                            #self.ry_axis = scaled_value
                            #print("6  cmd -- ABS RY R3", code, self.ry_axis) 

                        elif code == 'ABS_HAT0X':
                            self.rz_axis = scaled_value
                            
                        # Reset counter after update
                        event_counter[code] = 0

                gripper_command = Float64()                
                # Handle button events immediately
                if 'BTN_TL' in latest_events:
                    self.left = bool(latest_events['BTN_TL'])
                    if self.left and not self.prev_left:  # only execute at pression
                        self._reset_cmds()
                        print("\n LB --> OPEN GRIPPER", code, self.rz_axis)

                        ############ OPEN GRIPPER CODE FOR  REAL ROBOT & URSIM ############
                        # if self.service_client:
                        #     self.call_set_io_service(7, 0.0)  # annulla altro
                        #     self.call_set_io_service(6, 1.0)  # Chiamare il servizio per aprire il gripper

                        gripper_command.data = 0.0 # 0.0 = OPEN

                        # ---------- OPEN GRIPPER CODE FOR MUJOCO SIMULATION ----------
                        # self.node._mujoco_gripper_publisher.publish(gripper_command) # to PUB directly on mujoco

                        # ---------- OPEN GRIPPER CODE FOR Real Robot Case ----------
                        self.node._gym_gripper_publisher.publish(gripper_command) # to PUB INdirectly on Ros stepping through Gym


                if 'BTN_TR' in latest_events:
                    self.right = bool(latest_events['BTN_TR'])
                    if self.right and not self.prev_right:  # Esegui solo alla pressione
                        self._reset_cmds()
                        print("\n RB --> CLOSE GRIPPER", code, self.rz_axis)

                        ############ CLOSE GRIPPER CODE FOR  REAL ROBOT & URSIM ############
                        # if self.service_client:
                        #     self.call_set_io_service(6, 0.0)  # annulla altro
                        #     self.call_set_io_service(7, 1.0)  # Chiamare il servizio per chiudere il gripper

                        gripper_command.data = 239.0 # 255.0 = CLOSED

                        # ---------- CLOSE GRIPPER CODE FOR MUJOCO SIMULATION ----------
                        # self.node._mujoco_gripper_publisher.publish(gripper_command) # to PUB directly on mujoco


                        # ---------- CLOSE GRIPPER CODE FOR Real Robot Case ----------
                        self.node._gym_gripper_publisher.publish(gripper_command) # to PUB INdirectly on ROS stepping through Gym


                    self.prev_right = self.right  # Aggiorna lo stato precedente

                
            except inputs.UnpluggedError:
                print("No controller found. Retrying...")
                time.sleep(1)

            # handle/filter commands
            deadzone = 0.1 
            self.expert_a = np.zeros(6)

            self.expert_a[0] = self.x_axis if abs(self.x_axis) > deadzone else 0
            self.expert_a[1] = self.y_axis if abs(self.y_axis) > deadzone else 0
            self.expert_a[2] = self.z_axis if abs(self.z_axis) > deadzone else 0

            # Apply deadzone to rotation control -- Ignored
            self.expert_a[3] = self.rx_axis if abs(self.rx_axis) > deadzone else 0
            self.expert_a[4] = self.ry_axis if abs(self.ry_axis) > deadzone else 0
            self.expert_a[5] = self.rz_axis if abs(self.rz_axis) > deadzone else 0

            self.intervened = False
            if np.linalg.norm(self.expert_a) > 0.001 or self.left or self.right:
                self.intervened = True

            # self._reset_cmds() # L'ORIGINE DI TUTTI I MALI: -->  ANCHE PER POLICY COMMENTO E AUMENTO DEADZONE A 0,1!!!

            # print("------- INTERVENUTO VAR = ", self.intervened)
            if self.gripper_enabled:
                if self.left:  # close gripper
                    self.gripper_intervened = True
                elif self.right:  # open gripper
                    self.gripper_intervened = True

            if self.intervened: ### decommentare... (SOL dei MALI)
                print("")

            time.sleep(0.001)

    def call_set_io_service(self, pin, state):
        if not self.node:
            print("Nodo non inizializzato correttamente.")
            return
        request = ur_msgs.srv.SetIO.Request()
        request.fun = 1
        request.pin = pin
        request.state = state
        future = self.service_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            print(f"Service call succeeded: {future.result()}")
        else:
            print(f"Service call failed: {future.exception()}")

class AdmittanceControllerNode(Node): 
    def __init__(self):
        super().__init__('joystick_node')

        self.current_pose = None

        # IO_Gripper service client to move Gripper on REAL ROBOT/sim_Docker 
        service_client = self.create_client(ur_msgs.srv.SetIO, '/io_and_status_controller/set_io')
        # Pass client to Class constructor
        self.joystick = JoystickIntervention(service_client=service_client, node=self)  

        # Robot's Pose subscriber
        self.subscription = self.create_subscription(PoseStamped,'/admittance_controller/w_T_ee',self.pose_callback,10) 

        # publisher to control the gripper indirectly on ROS stepping through GYM
        self._gym_gripper_publisher = self.create_publisher(Float64, 'controller_intervention_gripper', 10) 
        # publisher to control the tcp indirectly on ROS stepping through GYM, only send offsets
        self.offset_publisher = self.create_publisher(Vector3, 'controller_intervention_offset', 10)

        # pub al topic del controllore per inviare il goal desiderato DIRETTAMENTE(i.e. pos attuale + offset dato da input joystick)
        # self.publisher = self.create_publisher(PoseStamped, '/admittance_controller/target_pose', 10)
        # # pub al topic per controllare il gripper su MUJOCO DIRETTAMENTE
        # self._mujoco_gripper_publisher = self.create_publisher(Float64, 'mujoco_ros/gripper_command', 10) 

        timer_period = 0.05  # seconds
        self.pub_timer = self.create_timer(timer_period, self.check_and_publish_new_pose)
        self.get_logger().info('Admittance Controller Node con joystick pronto.') 

    def pose_callback(self, msg):
        """Callback per aggiornare la posa corrente."""
        self.current_pose = msg

    def check_and_publish_new_pose(self):
        """Pubblica una nuova posa se cè nuovo input joystick."""
        if self.current_pose is None:
            self.get_logger().warn("Nessuna posa corrente disponibile. Attendo aggiornamenti...")
            return
        
        if  self.joystick.gripper_intervened:
            self.get_logger().info(' GRIPPER INTERVENED = TRUE') 
            self.joystick.gripper_intervened = False # reset flag
            return  # don't publish tcp pose if only gripper is commanded    
        
        if not self.joystick.intervened:
            return  
        
        #### RIMUOVO PUBLISHER DIRETTO AL ROBOT
        # new_pose = PoseStamped()
        # new_pose.header.stamp = self.get_clock().now().to_msg()
        # new_pose.header.frame_id = 'base_link'

        # new_pose.pose.position.x = self.current_pose.pose.position.x + self.joystick.expert_a[0]
        # new_pose.pose.position.y = self.current_pose.pose.position.y + self.joystick.expert_a[1]
        # new_pose.pose.position.z = self.current_pose.pose.position.z + self.joystick.expert_a[2]
        # new_pose.pose.orientation = self.current_pose.pose.orientation

        # self.publisher.publish(new_pose) #### RIMUOVO PUBLISHER DIRETTO AL ROBOT --> nel caso di Demo pubblica già Gym e non serve doppio publisher

        offset_msg = Vector3()
        offset_msg.x = self.joystick.expert_a[0]
        offset_msg.y = self.joystick.expert_a[1]
        offset_msg.z = self.joystick.expert_a[2]
        print("offset aggiunti = ", self.joystick.expert_a[0], self.joystick.expert_a[1], self.joystick.expert_a[2])

        self.offset_publisher.publish(offset_msg)
        self.get_logger().info(f"*********** Offset pubblicato separatamente: x={offset_msg.x}, y={offset_msg.y}, z={offset_msg.z}")
     
def main(args=None):

    rclpy.init(args=args)
    node = AdmittanceControllerNode()

    try:
        while rclpy.ok():
            rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Interruzione da tastiera, arresto del nodo.")

    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
