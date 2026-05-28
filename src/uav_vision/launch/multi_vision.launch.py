from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    num_uavs_str = LaunchConfiguration('robots').perform(context)
    show_gui_str = LaunchConfiguration('show_gui').perform(context)
    try:
        num_uavs = int(num_uavs_str)
    except ValueError:
        num_uavs = 1
    show_gui = show_gui_str.lower() == 'true'

    nodes_to_start = []

    for i in range(1, num_uavs + 1):
        uav_name = f'uav{i}'
        
        # YOLO Detector
        vision_node = Node(
            package='uav_vision',
            executable='yolo_detector',
            name=f'yolo_{uav_name}',
            namespace=uav_name,
            parameters=[{
                'input_topic': f'/{uav_name}/camera/image_raw',
                'show_gui': show_gui,
            }],
            output='screen'
        )
        nodes_to_start.append(vision_node)

        # Georeferencer (convierte pixeles a GPS)
        georef_node = Node(
            package='uav_vision',
            executable='georeferencer',
            name=f'georef_{uav_name}',
            namespace=uav_name,
            output='screen'
        )
        nodes_to_start.append(georef_node)

    # Map Consolidator (un solo nodo central que escucha a TODOS los drones)
    consolidator_node = Node(
        package='uav_vision',
        executable='map_consolidator',
        name='map_consolidator',
        parameters=[{
            'num_uavs': num_uavs,
        }],
        output='screen'
    )
    nodes_to_start.append(consolidator_node)

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots',
            default_value='1',
            description='Number of YOLO vision nodes to start'
        ),
        DeclareLaunchArgument(
            'show_gui',
            default_value='false',
            description='Show OpenCV GUI windows (true/false)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
