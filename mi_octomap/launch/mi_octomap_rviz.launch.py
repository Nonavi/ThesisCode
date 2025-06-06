from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'frame_id': 'world'},
                {'resolution': 0.01},
                {'publish_voxel_map': True},
                {'max_range': 0.5},
                {'sensor_model.max_range': 0.5},
                {'sensor_model.hit': 0.8},
                {'sensor_model.miss': 0.05},  # muy conservador al borrar
                #{'prob_hit': 0.9},           # Actualmente dice que se estan ignorando porque tiene prioridad el sensor_model...
                #{'prob_miss': 0.1},          # Actualmente dice que se estan ignorando porque tiene prioridad el sensor_model...
                {'prob_hit': 0.9},            # requiere confianza para marcar ocupado
                {'prob_miss': 0.1},           # necesita muchas misses para liberar
                {'thres_min': 0.12},          # mínimo para marcar como libre
                {'thres_max': 0.97},          # máximo para considerar ocupado
                {'filter_speckles': True},    # limpia puntos flotantes de ruido
            ],
            # Parámetro correcto del tópico de entrada:
            remappings=[
                ('/cloud_in', '/camera/camera/depth/color/points')
            ]
        )
    ])
