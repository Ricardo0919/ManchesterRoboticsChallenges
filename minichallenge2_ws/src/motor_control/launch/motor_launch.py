from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Obtener la dirección del archivo YAML de configuración
    config = os.path.join(
        get_package_share_directory('motor_control'),  # Obtener el directorio del paquete motor_control
        'config',  # Subcarpeta donde está el archivo de parámetros
        'params.yaml'  # Archivo YAML con parámetros de configuración
    )

    # Nodo 1 del sistema de control del motor
    motor_node_1 = Node(
        name="motor_sys_1",  # Nombre del nodo
        package='motor_control',  # Paquete del nodo
        executable='dc_motor',  # Ejecutable que se va a ejecutar
        emulate_tty=True,  # Emula una terminal para la salida
        output='screen',  # Redirigir la salida estándar a la pantalla
        namespace="group1",  # Namespace del nodo
        parameters=[config]  # Cargar los parámetros desde el archivo YAML
    )

    # Nodo 2 del sistema de control del motor
    motor_node_2 = Node(
        name="motor_sys_2",
        package='motor_control',
        executable='dc_motor',
        emulate_tty=True,
        output='screen',
        namespace="group2",  # Namespace diferente para el segundo grupo
        parameters=[config]
    )
    
    # Nodo 1 generador de puntos de referencia para el motor
    sp_node_1 = Node(
        name="sp_gen_1",
        package='motor_control',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        namespace="group1",  # Se coloca en el mismo grupo que el motor correspondiente
    )

    # Nodo 2 generador de puntos de referencia para el motor
    sp_node_2 = Node(
        name="sp_gen_2",
        package='motor_control',
        executable='set_point',
        emulate_tty=True,
        output='screen',
        namespace="group2",  # Namespace diferente para el segundo grupo
    )
    
    # Nodo 1 del controlador
    ctrl_node_1 = Node(
        name="ctrl_1",
        package='motor_control',
        executable='controller',
        emulate_tty=True,
        output='screen',
        namespace="group1",  # Namespace para el primer grupo
        parameters=[config]  # Cargar los parámetros de configuración
    )

    # Nodo 2 del controlador
    ctrl_node_2 = Node(
        name="ctrl_2",
        package='motor_control',
        executable='controller',
        emulate_tty=True,
        output='screen',
        namespace="group2",  # Namespace para el segundo grupo
        parameters=[config]
    )
    
    # Nodo para visualizar gráficamente los datos de los motores en rqt_plot
    rqt_plot = Node(
        name='rqt_plot',
        package='rqt_plot',
        executable='rqt_plot',
        arguments=[  # Argumentos con los tópicos que se desean visualizar
            '/group1/set_point/data', 
            '/group1/motor_input_u/data', 
            '/group1/motor_speed_y/data',
            '/group2/set_point/data', 
            '/group2/motor_input_u/data', 
            '/group2/motor_speed_y/data'
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
    l_d = LaunchDescription([motor_node_1, motor_node_2, sp_node_1, sp_node_2, ctrl_node_1, ctrl_node_2, rqt_plot, rqt_graph, rqt_reconfigure])

    return l_d  # Retornar la descripción de lanzamiento
