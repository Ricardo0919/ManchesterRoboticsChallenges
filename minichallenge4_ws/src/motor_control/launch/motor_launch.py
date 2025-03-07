from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Nodo 1 generador de puntos de referencia para el motor
    Input = Node(
        name="Input",
        package='motor_control',
        executable='Input',
        emulate_tty=True,
        output='screen'
    )
    
    # Nodo para visualizar gráficamente los datos de los motores en rqt_plot
    rqt_plot = Node(
        name='rqt_plot',
        package='rqt_plot',
        executable='rqt_plot',
        arguments=[  # Argumentos con los tópicos que se desean visualizar
            '/set_point/data', 
            '/motor_output/data',
            '/motor_pwm/data'
        ]
    )
    
    # Nodo para mostrar el grafo de nodos de ROS en rqt_graph
    rqt_graph = Node(
        name='rqt_graph',
        package='rqt_graph',
        executable='rqt_graph'
    )

    # Nodo para configurar parámetros en tiempo real usando rqt_reconfigure
    rqt_reconfigure = Node(
        name='rqt_reconfigure',
        package='rqt_reconfigure',
        executable='rqt_reconfigure'
    )

    # Definir la descripción de lanzamiento con todos los nodos configurados
    l_d = LaunchDescription([Input, rqt_plot, rqt_graph, rqt_reconfigure])

    return l_d  # Retornar la descripción de lanzamiento
