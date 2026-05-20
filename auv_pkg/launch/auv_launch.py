from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
    return LaunchDescription([
 
        Node(
            package='auv_pkg',
            executable='serial_monitoring_safety_and_bucket',
            name='serial_bridge_and_color_detection_node',
            output='screen',
        ),
 
        Node(
            package='auv_pkg',
            executable='object_detection_new',
            name='node_object_detection_new',
            output='screen',
        ),

        Node(
            package='auv_pkg',
            executable='accumulator_new',
            name='accumulator_subscriber',
            output='screen',
        ),
 
    ])
 