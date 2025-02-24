from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    #Get the address of the YAML File
    config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'params.yaml'
    )

    motor_node_1 = Node(
        name="motor_sys_1",
        package='motor_control',
        executable='dc_motor',
        emulate_tty=True,
        output='screen',
        namespace="group1",
        parameters=[config]
    )

    motor_node_2 = Node(
        name="motor_sys_2",
        package='motor_control',
        executable='dc_motor',
        emulate_tty=True,
        output='screen',
        namespace="group2",
        parameters=[config]
    )
    
    sp_node_1 = Node(
        name="sp_gen_1",
        package='motor_control',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        namespace="group1",
    )

    sp_node_2 = Node(
        name="sp_gen_2",
        package='motor_control',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        namespace="group2",
    )
    
    ctrl_node_1 = Node(
        name="ctrl_1",
        package='motor_control',
        executable='controller',
        emulate_tty=True,
        output='screen',
        namespace="group1",
        parameters=[config]
    )

    ctrl_node_2 = Node(
        name="ctrl_2",
        package='motor_control',
        executable='controller',
        emulate_tty=True,
        output='screen',
        namespace="group2",
        parameters=[config]
    )
    
    rqt_plot = Node(
        name='rqt_plot',
        package='rqt_plot',
        executable='rqt_plot',
        arguments=[
            '/group1/set_point/data', 
            '/group1/motor_input_u/data', 
            '/group1/motor_speed_y/data',
            '/group2/set_point/data', 
            '/group2/motor_input_u/data', 
            '/group2/motor_speed_y/data'
        ]
    )
    
    rqt_graph = Node(
        name='rqt_graph',
        package='rqt_graph',
        executable='rqt_graph'
    )

    rqt_reconfigure = Node(
        name='rqt_reconfigure',
        package='rqt_reconfigure',
        executable='rqt_reconfigure'
    )

    
    l_d = LaunchDescription([motor_node_1, motor_node_2, sp_node_1, sp_node_2, ctrl_node_1, ctrl_node_2, rqt_plot, rqt_graph, rqt_reconfigure])

    return l_d