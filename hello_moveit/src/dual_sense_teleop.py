#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from moveit_commander import MoveGroupCommander  # Asegúrate de tener la versión para ROS2

class DualSenseTeleop(Node):
    def __init__(self):
        super().__init__('dual_sense_teleop')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Inicializa el MoveGroupCommander para el grupo de planificación correspondiente
        self.move_group = MoveGroupCommander("tu_grupo_de_planificacion")  # Reemplaza con el nombre adecuado
        
        # Escalas para ajustar la velocidad y precisión del movimiento
        self.SCALE_TRANSLATION = 0.01  
        self.SCALE_ROTATION = 0.05     

    def joy_callback(self, msg: Joy):
        # Obtén la pose actual del robot
        current_pose = self.move_group.get_current_pose().pose

        # Mapea los ejes del DualSense a los movimientos deseados
        current_pose.position.x += msg.axes[0] * self.SCALE_TRANSLATION
        current_pose.position.y += msg.axes[1] * self.SCALE_TRANSLATION
        current_pose.position.z += msg.axes[3] * self.SCALE_TRANSLATION

        # Para la orientación, podrías agregar la conversión necesaria según tus requerimientos
        # Ejemplo (pseudocódigo): current_pose.orientation = convertir_ejes_a_cuaternion(msg.axes[4], msg.axes[5])

        # Establece la nueva pose como goal y envía el movimiento
        self.move_group.set_pose_target(current_pose)
        plan = self.move_group.go(wait=True)
        self.move_group.stop()             # Para detener cualquier movimiento residual
        self.move_group.clear_pose_targets()

def main(args=None):
    rclpy.init(args=args)
    dual_sense_node = DualSenseTeleop()
    rclpy.spin(dual_sense_node)
    dual_sense_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
