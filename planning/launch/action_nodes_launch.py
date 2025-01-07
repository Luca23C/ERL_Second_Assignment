"""
Spawn Robot Description
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Definizione della configurazione del namespace


    # Definizione dei nodi
    move_cmd = Node(
        package='planning',
        executable='move_node',
        name='move_node',

        output='screen',
        parameters=[]
    )

    detect_cmd = Node(
        package='planning',
        executable='detect_node',
        name='detect_node',

        output='screen',
        parameters=[]
    )

    move_to_lowest_id = Node(
        package='planning',
        executable='move_to_lowest_id_node',
        name='move_to_lowest_id_node',

        output='screen',
        parameters=[]
    )

    dummy_cmd = Node(
        package='planning',
        executable='dummy_map_node',
        name='dummy_map_node',

        output='screen',
        parameters=[]
    )
    
    service_node = Node(
        package='ros2_aruco',
        executable='detect_service_node',
        name='detect_service_node',

        output='screen',
        parameters=[]
    )


    # Creazione del LaunchDescription
    return LaunchDescription([
        move_cmd,
        detect_cmd,
        move_to_lowest_id,
        service_node,
        #dummy_cmd,
    ])