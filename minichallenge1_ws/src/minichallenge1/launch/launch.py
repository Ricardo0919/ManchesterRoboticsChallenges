from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='minichallenge1',
            executable='signal_generator',
            name='signal_generator',
            output='screen'
        ),

        Node(
            package='minichallenge1',
            executable='process',
            name='process',
            output='screen'
        ),

        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot',
            arguments=['/signal/data', '/proc_signal/data']
        ),

        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph'
        ),

    ])