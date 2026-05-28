from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    num_uavs_str = LaunchConfiguration('robots').perform(context)
    try:
        num_uavs = int(num_uavs_str)
    except ValueError:
        num_uavs = 1

    nodes_to_start = []

    for i in range(1, num_uavs + 1):
        uav_name = f'uav{i}'
        
        patrol_node = Node(
            package='uav_control',
            executable='patrol',
            name=f'patrol_{uav_name}',
            parameters=[{
                'uav_id': i,
                'num_uavs': num_uavs,
            }],
            output='screen'
        )
        nodes_to_start.append(patrol_node)

    return nodes_to_start

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots',
            default_value='1',
            description='Number of UAVs to patrol'
        ),
        OpaqueFunction(function=launch_setup)
    ])
