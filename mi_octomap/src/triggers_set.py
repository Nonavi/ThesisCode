#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
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

        # Inicialmente activado
        self.enabled = True

        # Suscripción al tópico de fuerzas
        self.subscription = self.create_subscription(
            Vector3,
            '/trigger_force',
            self.force_callback,
            10
        )

        # Suscripción al flag de habilitación
        self.enable_subscription = self.create_subscription(
            Bool,
            '/trigger_enable',
            self.enable_callback,
            10
        )

        self.get_logger().info("Nodo de PS5 Trigger listo. Esperando Vector3 en /trigger_force")
        self.get_logger().info("Puede activar/desactivar el nodo enviando Bool a /trigger_enable")

    def force_callback(self, msg):
        if not self.enabled:
            '''# Si deshabilitado → forzar a 0
            self.ds.triggerL.setForce(1, 0)
            self.ds.triggerR.setForce(1, 0)
            self.get_logger().info("Modo desactivado → Fuerzas = 0")'''
            return

        # Normal
        force_left = max(0, min(255, int(msg.y)))
        force_right = max(0, min(255, int(msg.x)))

        self.ds.triggerL.setForce(1, force_left)
        self.ds.triggerR.setForce(1, force_right)

        self.get_logger().info(f"Fuerzas aplicadas - Izq: {force_left} | Der: {force_right}")

    def enable_callback(self, msg):
        self.enabled = msg.data
        if not self.enabled:
            # Al desactivarse → poner triggers en 0
            self.ds.triggerL.setForce(1, 0)
            self.ds.triggerR.setForce(1, 0)
            self.get_logger().info("TRIGGERS DESACTIVADOS")
        else:
            self.get_logger().info("TRIGGERS ACTIVADOS → retomando mensajes de /trigger_force")

    def destroy_node(self):
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
