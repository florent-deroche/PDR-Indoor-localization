from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ble_origin_broadcaster',
            # [X, Y, Z, Yaw, Pitch, Roll, Parent(SLAM), Enfant(BLE)]
            # il faut démarrer le robot sur l'origine de notre repère
            arguments=['0', '0', '0.0', '0.0', '0.0', '0.0', 'odom', 'ble_origin']
        ),
        
        Node(
            package='ble_localization',
            executable='trilateration_node',
            name='ble_trilateration_node',
            output='screen'
        ),
        
        Node(
            package='sensor_fusion',
            executable='fusion_node',
            name='kalman_fusion_node',
            output='screen'
        )
    ])