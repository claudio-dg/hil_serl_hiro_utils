#!/usr/bin/env python3
"""VERSIONE FUNZIONANTE X MUOVERE UR CON GAMEPAD XBOX"""
# NOTA: questo nodo deve poi diventare un wrapper gym come quello originale per wrappare
# poi l'env e aggiungere questa feature.. quindi bisogna cambiare struttura facendo vari metodi step action ecc..
# guardare wrappers.py per capire come fare
## aggiunta gestione del gripper tramite bumper sinistro e destro del gamepad
# sia per simulazione MUJOCO che per caso reale con input IO UR


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
                'ABS_X': 0.1*9, ### x sim originale: 0.1 ___ x robot reale : 0.05
                'ABS_Y': 0.1*9, ### x sim originale: 0.1 ___ x robot reale : 0.05
                'ABS_RX': 0.3, ##non usato 
                'ABS_RY': 0.3, ##non usato
                'ABS_Z': 0.015*5, #### originale 0.05 --> modifico per UR --> movimenti esagerati asse z, provo a limitarli con valori più bassi --> 0.015
                'ABS_RZ': 0.015*5, ## sim 0.015 .. reale abbasso un pelo a 0.013 forse? di più mi sa che poi funzion peggio prendendo un comando ogni 10 e pericoloso perchè robot si muove poi a improvviso & imprevedibile
                'ABS_HAT0X': 0.3,
            }
        ),
    }
    def __init__(self,  action_indices=None, controller_type=ControllerType.XBOX, service_client=None, node=None):
        
        self.service_client = service_client  # Inizializzare il client del servizio
        self.node = node  # Inizializzare il nodo

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
        self.prev_left = False  # Stato precedente del pulsante BTN_TL per evitare esecuzione doppia comando (alla pressione a al rilascio)
        self.prev_right = False  # Stato precedente del pulsante BTN_TR

        
        
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

                        normalized_value = current_value / (resolution / 2) # fa diviso Res/2 perchè poi scala in modo da avere da  -TOT/2 a +TOT/2 anziche da 0 a TOT (una roba così)
                        scaled_value = normalized_value * self.controller_config.scale[code]

                        # sti nomi ABS_Y HAT0X ecc sono mezzi a caso dati dal lettore del gamepad non da sto codice -- vedi my_inpuptGamepad
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

                gripper_command = Float64()                
                # Handle button events immediately
                if 'BTN_TL' in latest_events:
                    self.left = bool(latest_events['BTN_TL'])
                    if self.left and not self.prev_left:  # Esegui solo alla pressione
                        self._reset_cmds()
                        print("\n LB --> OPEN GRIPPER", code, self.rz_axis)

                        ############ OPEN GRIPPER CODE FOR  REAL ROBOT & URSIM ############
                        
                        # if self.service_client:
                        #     self.call_set_io_service(7, 0.0)  # annulla altro
                        #     self.call_set_io_service(6, 1.0)  # Chiamare il servizio per aprire il gripper

                        ############ OPEN GRIPPER CODE FOR  REAL ROBOT & URSIM ############

                        # ---------- OPEN GRIPPER CODE FOR MUJOCO SIMULATION ----------

                        gripper_command.data = 0.0 # 0.0 = aperto
                        self.node._mujoco_gripper_publisher.publish(gripper_command)

                        # ---------- OPEN GRIPPER CODE FOR MUJOCO SIMULATION ----------

                    self.prev_left = self.left  # Aggiorna lo stato precedente

                if 'BTN_TR' in latest_events:
                    self.right = bool(latest_events['BTN_TR'])
                    if self.right and not self.prev_right:  # Esegui solo alla pressione
                        self._reset_cmds()
                        print("\n RB --> CLOSE GRIPPER", code, self.rz_axis)

                        ############ CLOSE GRIPPER CODE FOR  REAL ROBOT & URSIM ############

                        # if self.service_client:
                        #     self.call_set_io_service(6, 0.0)  # annulla altro
                        #     self.call_set_io_service(7, 1.0)  # Chiamare il servizio per chiudere il gripper

                        ############ CLOSE GRIPPER CODE FOR  REAL ROBOT & URSIM ############

                        # ---------- CLOSE GRIPPER CODE FOR MUJOCO SIMULATION ----------

                        gripper_command.data = 239.0 # 255.0 = chiuso
                        self.node._mujoco_gripper_publisher.publish(gripper_command)

                        # ---------- CLOSE GRIPPER CODE FOR MUJOCO SIMULATION ----------

                    self.prev_right = self.right  # Aggiorna lo stato precedente

                
            except inputs.UnpluggedError:
                print("No controller found. Retrying...")
                time.sleep(1)

            #provo a mettere qua gestione/filtro dei comandi
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
            if np.linalg.norm(self.expert_a) > 0.001 or self.left or self.right:
                self.intervened = True

            # print("------- INTERVENUTO VAR = ", self.intervened)
            if self.gripper_enabled:
                if self.left:  # close gripper
                    # gripper_action = np.random.uniform(-1, -0.9, size=(1,))
                    self.gripper_intervened = True
                elif self.right:  # open gripper
                    # gripper_action = np.random.uniform(0.9, 1, size=(1,))
                    self.gripper_intervened = True
                # else:
                #     gripper_action = np.zeros((1,))
                # self.expert_a = np.concatenate((self.expert_a, gripper_action), axis=0)
                # expert_a[:6] += np.random.uniform(-0.5, 0.5, size=6)

            if self.action_indices is not None:
                filtered_expert_a = np.zeros_like(self.expert_a)
                filtered_expert_a[self.action_indices] = self.expert_a[self.action_indices] # spiega a riga  468: specifica quali dimensioni considerare e quali no
                # filtered_expert_a = expert_a[self.action_indices]
                self.expert_a = filtered_expert_a

            if self.intervened: ### decommentare
                #### decommenta per debug controllare inputs ###
                print("[DEBUG] Expert action intervened. Values: \n X = ", self.expert_a[0], " Y = ", self.expert_a[1], " Z = ", self.expert_a[2], " RX = ", self.expert_a[3], " RY = ", self.expert_a[4], " Rz = ", self.expert_a[5]) #, "gripper = ", self.expert_a[6])  # Debug print             
                # print("")
                #return expert_a, True 

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

        # IO_Gripper service client per muovere il Gripper su ROBOT REALE / su sim_Docker (per quello su mujoco uso publisher sotto)
        service_client = self.create_client(ur_msgs.srv.SetIO, '/io_and_status_controller/set_io')
        # COMMENTATO PER TEST SU SIMULAZIONE CHE NON HA TALE SERVIZIO #
        # while not service_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('IO Service not available, waiting again...')
        # self.get_logger().info('IO Service client ready.')
        self.joystick = JoystickIntervention(service_client=service_client, node=self)  # Passare il client al costruttore

     
        # sub al topic del robot per leggere pos attuale end effector
        self.subscription = self.create_subscription(PoseStamped,'/admittance_controller/w_T_ee',self.pose_callback,10) 

        # pub al topic del controllore per inviare il goal desiderato (i.e. pos attuale + offset dato da input joystick)
        self.publisher = self.create_publisher(PoseStamped, '/admittance_controller/target_pose', 10)

        # pub al topic per controllare il gripper su MUJOCO
        self._mujoco_gripper_publisher = self.create_publisher(Float64, 'mujoco_ros/gripper_command', 10) 

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
            self.joystick.gripper_intervened = False # reset flag
            return  # Non pubblicare pos nuove dell' end eff se comanda solo apertura/chiusura gripper    

        
        if not self.joystick.intervened:
            return  # Non pubblicare se non ci sono nuovi comandi
        
        
        new_pose = PoseStamped()
        new_pose.header.stamp = self.get_clock().now().to_msg()
        new_pose.header.frame_id = 'base_link'

        new_pose.pose.position.x = self.current_pose.pose.position.x + self.joystick.expert_a[0]
        new_pose.pose.position.y = self.current_pose.pose.position.y + self.joystick.expert_a[1]
        new_pose.pose.position.z = self.current_pose.pose.position.z + self.joystick.expert_a[2]
        new_pose.pose.orientation = self.current_pose.pose.orientation

        self.publisher.publish(new_pose)
        self.get_logger().info(f" ### Nuova posa pubblicata: x={new_pose.pose.position.x}, y={new_pose.pose.position.y}, z={new_pose.pose.position.z}")

     
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
