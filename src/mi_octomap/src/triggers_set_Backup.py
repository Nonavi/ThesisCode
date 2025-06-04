#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from pydualsense import pydualsense, TriggerModes


class TriggerForceNode(Node):
    def __init__(self):
        super().__init__('trigger_force_node')

        # Inicializar DualSense
        self.ds = pydualsense()
        self.ds.init()

        # Establecer modo RIGID para los dos gatillos
        self.ds.triggerL.setMode(TriggerModes.Rigid)
        self.ds.triggerR.setMode(TriggerModes.Rigid)

        # Suscripción al tópico para recibir fuerza
        self.subscription = self.create_subscription(
            Int32,
            '/trigger_force',
            self.force_callback,
            10
        )

        self.get_logger().info("Nodo de PS5 Trigger listo. Escuchando en /trigger_force")

    def force_callback(self, msg):
        force = int(msg.data)
        force = max(0, min(255, force))  # Asegurar que esté en el rango

        # Establecer fuerza en ambos gatillos
        self.ds.triggerL.setForce(1, force)
        self.ds.triggerR.setForce(1, force)
        self.get_logger().info(f"Fuerza recibida: {force}")

    def destroy_node(self):
        # Al apagar, apagar los gatillos y cerrar el control
        self.get_logger().info("Apagando control DualSense...")
        self.ds.triggerL.setForce(0)
        self.ds.triggerR.setForce(0)
        self.ds.triggerL.setMode(TriggerModes.Off)
        self.ds.triggerR.setMode(TriggerModes.Off)
        self.ds.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TriggerForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción detectada.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
