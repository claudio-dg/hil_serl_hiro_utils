#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random


class AdmittanceControllerNode(Node):
    def __init__(self):
        super().__init__('admittance_controller_node')

        # Variabile per memorizzare la posa corrente
        self.current_pose = None
        
        self.my_offset = 0.15

        # Subscriber per leggere la posa attuale
        self.subscription = self.create_subscription(
            PoseStamped,
            '/admittance_controller/w_T_ee',
            self.pose_callback,
            10
        )

        # Publisher per inviare la nuova posa con offset
        self.publisher = self.create_publisher(PoseStamped, '/admittance_controller/target_pose', 10)

        # Timer per pubblicare periodicamente una nuova posa con offset
        self.timer = self.create_timer(5.0, self.publish_new_pose)  # Pubblica ogni 1 secondo

        self.get_logger().info('Admittance Controller Node è pronto.')

    def pose_callback(self, msg):
        """Callback per aggiornare la posa corrente."""
        self.current_pose = msg
        #self.get_logger().info(f"Posa corrente aggiornata: {msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")

    def publish_new_pose(self):
        """Pubblica una nuova posa con un offset basato sulla posa corrente."""
        if self.current_pose is None:
            self.get_logger().warn("Nessuna posa corrente disponibile. Attendo aggiornamenti dal subscriber...")
            return
            
        self.get_logger().info(f" Posa attuale var globale: : {self.current_pose.pose.position.x}, {self.current_pose.pose.position.y}, {self.current_pose.pose.position.z}")

        # Crea un nuovo messaggio di tipo PoseStamped
        new_pose = PoseStamped()
        new_pose.header.stamp = self.get_clock().now().to_msg()
        #new_pose.header.frame_id = self.current_pose.header.frame_id
        new_pose.header.frame_id = 'base_link'

        # Aggiungi un offset casuale alla posa corrente
        self.my_offset = - self.my_offset
        new_pose.pose.position.x = self.current_pose.pose.position.x 
        new_pose.pose.position.y = self.current_pose.pose.position.y 
        new_pose.pose.position.z = self.current_pose.pose.position.z + self.my_offset
        
        offset = 0.1  # Offset fisso o casuale
        #new_pose.pose.position.x = self.current_pose.pose.position.x + random.uniform(-offset, offset)
        #new_pose.pose.position.y = self.current_pose.pose.position.y + random.uniform(-offset, offset)
        #new_pose.pose.position.z = self.current_pose.pose.position.z + random.uniform(-offset, offset)

        # Copia orientamento dalla posa corrente
        new_pose.pose.orientation = self.current_pose.pose.orientation

        # Pubblica la nuova posa
        self.publisher.publish(new_pose)
        self.get_logger().info(f"\n\n Nuova posa pubblicata: {new_pose.pose.position.x}, {new_pose.pose.position.y}, {new_pose.pose.position.z}")


def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceControllerNode()

    rclpy.spin(node)

    # Distruggi il nodo dopo l'uscita dal ciclo
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

