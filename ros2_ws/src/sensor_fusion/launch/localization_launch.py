from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Le nœud qui écoute le MQTT et fait la trilatération
        Node(
            package='ble_localization',
            executable='trilateration_node',
            name='ble_trilateration_node',
            output='screen'
        ),
        
        # 2. Le nœud qui fusionne le BLE et l'Odométrie avec Kalman
        Node(
            package='sensor_fusion',
            executable='fusion_node',
            name='kalman_fusion_node',
            output='screen'
        )
    ])