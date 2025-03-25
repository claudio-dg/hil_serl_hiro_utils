import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import inputs
import threading
import time

class JoystickIntervention():
    def __init__(self):
        self.x_axis = 0
        self.y_axis = 0
        self.z_axis = 0
        self.running = True
        self.thread = threading.Thread(target=self._read_gamepad)
        self.thread.daemon = True
        self.thread.start()
        self.new_command = False  # Flag per indicare nuovi comandi

    def _read_gamepad(self):
        useful_codes = ['ABS_X', 'ABS_Y', 'ABS_RZ', 'ABS_Z']
        event_counter = {code: 0 for code in useful_codes}

        while self.running:
            try:
                events = inputs.get_gamepad()
                latest_events = {}
                for event in events:
                    latest_events[event.code] = event.state
                for code in useful_codes:
                    if code in latest_events:
                        event_counter[code] += 1
                        current_value = latest_events[code]
                        resolution = 65536 if 'ABS_' in code else 256
                        normalized_value = current_value / (resolution / 2)

                        if code == 'ABS_Y':
                            self.x_axis = normalized_value * -0.1
                        elif code == 'ABS_X':
                            self.y_axis = normalized_value * -0.1
                        elif code == 'ABS_RZ':
                            self.z_axis = normalized_value * 0.05
                        elif code == 'ABS_Z':
                            self.z_axis = -normalized_value * 0.05

                        self.new_command = True  # Imposta il flag per nuovi comandi
                        event_counter[code] = 0

            except inputs.UnpluggedError:
                print("No controller found. Retrying...")
                time.sleep(1)

    def get_offsets(self):
        offsets = self.x_axis, self.y_axis, self.z_axis
        if self.new_command:
            self.new_command = False  # Resetta il flag dopo aver letto il comando
            return offsets, True
        return offsets, False


class AdmittanceControllerNode(Node):
    def __init__(self):
        super().__init__('admittance_controller_node')

        self.current_pose = None
        self.joystick = JoystickIntervention()  ################
     
        self.subscription = self.create_subscription( 
            PoseStamped,
            '/admittance_controller/w_T_ee',
            self.pose_callback,
            10
        ) 

        self.publisher = self.create_publisher(PoseStamped, '/admittance_controller/target_pose', 10) 

        self.get_logger().info('Admittance Controller Node con joystick pronto.') 

    def pose_callback(self, msg):
        """Callback per aggiornare la posa corrente."""
        self.current_pose = msg

    def check_and_publish_new_pose(self):
        """Pubblica una nuova posa se cè nuovo input joystick."""
        if self.current_pose is None:
            self.get_logger().warn("Nessuna posa corrente disponibile. Attendo aggiornamenti...")
            return

        offsets, new_command = self.joystick.get_offsets()
        if not new_command:
            return  # Non pubblicare se non ci sono nuovi comandi

        x_offset, y_offset, z_offset = offsets
        new_pose = PoseStamped()
        new_pose.header.stamp = self.get_clock().now().to_msg()
        new_pose.header.frame_id = 'base_link'

        new_pose.pose.position.x = self.current_pose.pose.position.x + x_offset
        new_pose.pose.position.y = self.current_pose.pose.position.y + y_offset
        new_pose.pose.position.z = self.current_pose.pose.position.z + z_offset
        new_pose.pose.orientation = self.current_pose.pose.orientation

        self.publisher.publish(new_pose)
        self.get_logger().info(f"Nuova posa pubblicata: x={new_pose.pose.position.x}, y={new_pose.pose.position.y}, z={new_pose.pose.position.z}")


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


if __name__ == '__main__':
    main()

