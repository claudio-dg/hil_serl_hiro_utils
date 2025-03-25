#!/usr/bin/env python3
"""VERSIONE 2 X alternare  GAMEPAD XBOX e waypoint FUNZIONANTE"""
# Questo è test per vedere se funziona alternanza input ma poi gestione di
#alternanza non sarà gestita così, bensì sarà gestita attraverso i wrapper dell'env

from __future__ import print_function
from dataclasses import dataclass
from enum import Enum
import inputs # attivare conda env hilserl per avere librerie
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

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
                'ABS_X': 0.1, ### x sim originale: 0.1 ___ x robot reale : 0.05
                'ABS_Y': 0.1, ### x sim originale: 0.1 ___ x robot reale : 0.05
                'ABS_RX': 0.3, ##non usato 
                'ABS_RY': 0.3, ##non usato
                'ABS_Z': 0.015, #### originale 0.05 --> modifico per UR --> movimenti esagerati asse z, provo a limitarli con valori più bassi --> 0.015
                'ABS_RZ': 0.015, ## sim 0.015 .. reale abbasso un pelo a 0.013 forse? di più mi sa che poi funzion peggio prendendo un comando ogni 10 e pericoloso perchè robot si muove poi a improvviso & imprevedibile
                'ABS_HAT0X': 0.3,
            }
        ),
    }
    def __init__(self,  action_indices=None, controller_type=ControllerType.XBOX):
        
        self.gripper_enabled = True
        # if self.action_space.shape == (6,):
        #     self.gripper_enabled = False
        # we can set action_indices to choose which action to intervene on
        # e.g. action_indices=[0, 1, 2] will only intervene on the position control
        self.action_indices = action_indices 
        self.controller_type = controller_type
        self.controller_config = self.CONTROLLER_CONFIGS[controller_type]
        
        # Controller state
        self.x_axis = 0
        self.y_axis = 0
        self.z_axis = 0
        self.rx_axis = 0
        self.ry_axis = 0
        self.rz_axis = 0
        self.left = False   # Left bumper for close gripper
        self.right = False  # Right bumper for open gripper
        
        # Start controller reading thread
        self.running = True
        self.thread = threading.Thread(target=self._read_gamepad)
        self.thread.daemon = True
        self.thread.start()
        self.intervened = False  # Flag per indicare nuovi comandi 
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

                        normalized_value = current_value / (resolution / 2) # fa diviso Res/2 perchè poi scala in modo da avere da  -TOT/2 a +TOT/2 anziche da 0 a TOT (una roba così)
                        scaled_value = normalized_value * self.controller_config.scale[code]

                        if code == 'ABS_Y': # su giu cursore (avanti indietro robot) indietro -0.1 avanti +0.1

                            self.x_axis =   scaled_value                            
                            #print("1 cmd -- ABS Y = L3 UP_DOWN", code, self.x_axis)

                        elif code == 'ABS_X': # dx sx cursore.. destra max = -0.1 sx max = 0.1

                            self.y_axis =  scaled_value 
                            #print("2 cmd -- ABS X  = L3 RIGHT_LEFT ", code,  self.y_axis)

                        elif code == 'ABS_RZ': # grilleto DX (RT) 0=non premuto max = 0.4 premuto .. fa salire robot
                            
                            self.z_axis = scaled_value
                            #print("3 cmd -- ABS RZ = RT", code, self.z_axis)

                        elif code == 'ABS_Z': # grilleto LT max = -0.4 min 0

                            # Flip sign so this will go in the down direction
                            self.z_axis = -scaled_value
                            #print("4  cmd -- ABS Z = LT", code, self.z_axis)

                        ### ignoro cursore destro
                        ### elif code == 'ABS_RX': # cursore R3 destra sinistra .. ma non mi sembra sia mappato al robot qua.. comunque da -0.3  a 0.3
                            #self.rx_axis = scaled_value
                            #print("5  cmd -- ABS RX R3", code, self.rx_axis)

                        ### elif code == 'ABS_RY': # da -0.3 a 0.3
                            #self.ry_axis = scaled_value
                            #print("6  cmd -- ABS RY R3", code, self.ry_axis) 

                        elif code == 'ABS_HAT0X':
                            self.rz_axis = scaled_value
                            
                        ###self.new_command = True  # Imposta il flag per nuovi comandi ############
                        # Reset counter after update
                        event_counter[code] = 0

                
                # Handle button events immediately --> gestire sepratamente pubblicando su topic grippere VEDERE POI
                if 'BTN_TL' in latest_events:
                    self.left = bool(latest_events['BTN_TL'])
                    self._reset_cmds()
                    print("\n LB --> OPEN GRIPPER", code, self.rz_axis)
                if 'BTN_TR' in latest_events:
                    self.right = bool(latest_events['BTN_TR'])
                    self._reset_cmds()
                    print("\n RB --> CLOSE GRIPPER", code, self.rz_axis)
                
            except inputs.UnpluggedError:
                print("No controller found. Retrying...")
                time.sleep(1)

            # Get joystick action
            deadzone = 0.03 #0.03
            self.expert_a = np.zeros(6)
            #expert_a = np.zeros(6)

            self.expert_a[0] = self.x_axis if abs(self.x_axis) > deadzone else 0
            self.expert_a[1] = self.y_axis if abs(self.y_axis) > deadzone else 0
            self.expert_a[2] = self.z_axis if abs(self.z_axis) > deadzone else 0

            # Apply deadzone to rotation control -- ma non lin considera questi secondo me...
            self.expert_a[3] = self.rx_axis if abs(self.rx_axis) > deadzone else 0
            self.expert_a[4] = self.ry_axis if abs(self.ry_axis) > deadzone else 0
            self.expert_a[5] = self.rz_axis if abs(self.rz_axis) > deadzone else 0

            self._reset_cmds()

            self.intervened = False
            # print("INTERVENUTO VAR = ", intervened)
            if np.linalg.norm(self.expert_a) > 0.001 or self.left or self.right:
                self.intervened = True

            # print("------- INTERVENUTO VAR = ", self.intervened)
            if self.gripper_enabled:
                if self.left:  # close gripper
                    gripper_action = np.random.uniform(-1, -0.9, size=(1,))
                    self.intervened = True
                elif self.right:  # open gripper
                    gripper_action = np.random.uniform(0.9, 1, size=(1,))
                    self.intervened = True
                else:
                    gripper_action = np.zeros((1,))
                self.expert_a = np.concatenate((self.expert_a, gripper_action), axis=0)
                # expert_a[:6] += np.random.uniform(-0.5, 0.5, size=6)

            if self.action_indices is not None:
                filtered_expert_a = np.zeros_like(self.expert_a)
                filtered_expert_a[self.action_indices] = self.expert_a[self.action_indices] # spiega a riga  468: specifica quali dimensioni considerare e quali no
                # filtered_expert_a = expert_a[self.action_indices]
                self.expert_a = filtered_expert_a

            if self.intervened:
                #### decommenta per debug controllare inputs ###
                # print("[DEBUG] Expert action intervened. Values: \n X = ", self.expert_a[0], " Y = ", self.expert_a[1], " Z = ", self.expert_a[2], " RX = ", self.expert_a[3], " RY = ", self.expert_a[4], " Rz = ", self.expert_a[5], "gripper = ", self.expert_a[6])  # Debug print             
                print(".")
                #return expert_a, True 

 
class AdmittanceControllerNode(Node):
    def __init__(self):
        super().__init__('admittance_controller_node')

        self.current_pose = None
        self.joystick = JoystickIntervention()  ##     
        self.subscription = self.create_subscription( 
            PoseStamped,
            '/admittance_controller/w_T_ee',
            self.pose_callback,
            10
        )
        self.publisher = self.create_publisher(PoseStamped, '/admittance_controller/target_pose', 10)
        ############# added waypoints ############# 
        self.waypoints = [
            PoseStamped(), PoseStamped(), PoseStamped()
        ]
        self.init_waypoints()
        self.current_waypoint_idx = 0
        self.last_waypoint_time = time.time()
        self.waypoint_interval = 3.0  # Pubblica waypoint ogni 4 secondi
        ############# added waypoints ############# 
        self.get_logger().info('Admittance Controller Node con joystick pronto.') 

    def pose_callback(self, msg):
        """Callback per aggiornare la posa corrente."""
        self.current_pose = msg
        ## aggiorna la posa anche nella classe del joystick
        # self.joystick.update_current_pose(msg.pose)
    
    ############# added waypoints ############# 
    def init_waypoints(self):
        self.waypoints[0].pose.position.x, self.waypoints[0].pose.position.y, self.waypoints[0].pose.position.z = -0.874, 0.154, 0.230
        self.waypoints[1].pose.position.x, self.waypoints[1].pose.position.y, self.waypoints[1].pose.position.z = -0.874, -0.14, 0.230
        self.waypoints[2].pose.position.x, self.waypoints[2].pose.position.y, self.waypoints[2].pose.position.z = -0.654, -0.14, 0.230
        for wp in self.waypoints:
            wp.header.frame_id = 'base_link'
    ############# added waypoints ############# 

    def check_and_publish_new_pose(self):
        if self.current_pose is None:
            self.get_logger().warn("Nessuna posa corrente disponibile. Attendo aggiornamenti...")
            return
        
        new_pose = None

        if self.joystick.intervened:
            new_pose = PoseStamped()
            new_pose.header.stamp = self.get_clock().now().to_msg()
            new_pose.header.frame_id = 'base_link'
            new_pose.pose.position.x = self.current_pose.pose.position.x + self.joystick.expert_a[0]
            new_pose.pose.position.y = self.current_pose.pose.position.y + self.joystick.expert_a[1]
            new_pose.pose.position.z = self.current_pose.pose.position.z + self.joystick.expert_a[2]
            new_pose.pose.orientation = self.current_pose.pose.orientation
            self.publisher.publish(new_pose)
            self.get_logger().info(f"AAA Nuova posa pubblicata: x={new_pose.pose.position.x}, y={new_pose.pose.position.y}, z={new_pose.pose.position.z}")
            return
        
        else:
            if time.time() - self.last_waypoint_time >= self.waypoint_interval:
                new_pose = self.get_next_waypoint()
                new_pose.pose.orientation = self.current_pose.pose.orientation
                new_pose.header.stamp = self.get_clock().now().to_msg()
                self.publisher.publish(new_pose)
                self.last_waypoint_time = time.time()

        if new_pose is not None:
            self.get_logger().info(f"Nuova posa pubblicata: x={new_pose.pose.position.x}, y={new_pose.pose.position.y}, z={new_pose.pose.position.z}")

    def get_next_waypoint(self):
        waypoint = self.waypoints[self.current_waypoint_idx]
        self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
        return waypoint

     
def main(args=None):

    rclpy.init(args=args)
    node = AdmittanceControllerNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.check_and_publish_new_pose()

    except KeyboardInterrupt:
        node.get_logger().info("Interruzione da tastiera, arresto del nodo.")

    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
