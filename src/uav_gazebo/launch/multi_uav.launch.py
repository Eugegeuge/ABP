import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_dynamic_bridge_yaml(num_robots, output_path):
    """Generates a YAML file for ros_gz_bridge connecting topics for N UAVs."""
    bridge_config = []
    
    # Common Clock
    bridge_config.append({
        'ros_topic_name': '/clock',
        'gz_topic_name': '/clock',
        'ros_type_name': 'rosgraph_msgs/msg/Clock',
        'gz_type_name': 'gz.msgs.Clock',
        'direction': 'GZ_TO_ROS'
    })
    
    # Topics for each UAV
    for i in range(1, num_robots + 1):
        uav_name = f'uav{i}'
        
        bridge_config.extend([
            # Movement and Odometry (Model-based GZ topics)
            {
                'ros_topic_name': f'/{uav_name}/cmd_vel',
                'gz_topic_name': f'/model/{uav_name}/cmd_vel',
                'ros_type_name': 'geometry_msgs/msg/Twist',
                'gz_type_name': 'gz.msgs.Twist',
                'direction': 'ROS_TO_GZ'
            },
            {
                'ros_topic_name': f'/{uav_name}/odom',
                'gz_topic_name': f'/model/{uav_name}/odometry',
                'ros_type_name': 'nav_msgs/msg/Odometry',
                'gz_type_name': 'gz.msgs.Odometry',
                'direction': 'GZ_TO_ROS'
            },
            # Sensors (Prefix-based GZ topics)
            {
                'ros_topic_name': f'/{uav_name}/imu',
                'gz_topic_name': f'/{uav_name}/imu',
                'ros_type_name': 'sensor_msgs/msg/Imu',
                'gz_type_name': 'gz.msgs.IMU',
                'direction': 'GZ_TO_ROS'
            },
            {
                'ros_topic_name': f'/{uav_name}/gps',
                'gz_topic_name': f'/{uav_name}/gps',
                'ros_type_name': 'sensor_msgs/msg/NavSatFix',
                'gz_type_name': 'gz.msgs.NavSat',
                'direction': 'GZ_TO_ROS'
            },
            {
                'ros_topic_name': f'/{uav_name}/camera/image_raw',
                'gz_topic_name': f'/{uav_name}/camera/image_raw',
                'ros_type_name': 'sensor_msgs/msg/Image',
                'gz_type_name': 'gz.msgs.Image',
                'direction': 'GZ_TO_ROS'
            },
            {
                'ros_topic_name': f'/{uav_name}/scan',
                'gz_topic_name': f'/{uav_name}/scan',
                'ros_type_name': 'sensor_msgs/msg/LaserScan',
                'gz_type_name': 'gz.msgs.LaserScan',
                'direction': 'GZ_TO_ROS'
            },
            {
                'ros_topic_name': f'/{uav_name}/scan_altitude',
                'gz_topic_name': f'/{uav_name}/scan_altitude',
                'ros_type_name': 'sensor_msgs/msg/LaserScan',
                'gz_type_name': 'gz.msgs.LaserScan',
                'direction': 'GZ_TO_ROS'
            },
            # TF (GZ to ROS)
            {
                'ros_topic_name': '/tf',
                'gz_topic_name': f'/model/{uav_name}/tf',
                'ros_type_name': 'tf2_msgs/msg/TFMessage',
                'gz_type_name': 'gz.msgs.Pose_V',
                'direction': 'GZ_TO_ROS'
            }
        ])
        
    # Write to tmp file
    with open(output_path, 'w') as f:
        yaml.dump(bridge_config, f, default_flow_style=False)


def launch_setup(context, *args, **kwargs):
    num_robots_str = LaunchConfiguration('robots').perform(context)
    try:
        num_robots = int(num_robots_str)
    except ValueError:
        num_robots = 2

    pkg_uav_description = get_package_share_directory('uav_description')
    pkg_uav_gazebo = get_package_share_directory('uav_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Generate bridge yaml dynamically
    bridge_yaml_path = '/tmp/uav_dynamic_bridge.yaml'
    generate_dynamic_bridge_yaml(num_robots, bridge_yaml_path)

    nodes_to_start = []

    # 1. Gazebo Server and Client
    world_path = os.path.join(pkg_uav_gazebo, 'worlds', 'outdoor.world')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items()
    )
    nodes_to_start.append(gz_sim)

    # 2. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_yaml_path,
            'use_sim_time': True
        }],
        output='screen'
    )
    nodes_to_start.append(bridge)

    # 3. Spawn UAVs
    urdf_xacro_path = os.path.join(pkg_uav_description, 'urdf', 'uav.xacro')

    for i in range(1, num_robots + 1):
        uav_name = f'uav{i}'
        prefix = f'{uav_name}/'
        
        # Position them in a line
        x_pos = 0.0
        y_pos = (i - 1) * 2.0
        z_pos = 0.5

        # State Publisher
        uav_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=uav_name,
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_xacro_path, ' prefix:=', prefix]),
                'use_sim_time': True
            }],
            output='screen'
        )
        nodes_to_start.append(uav_state_publisher)

        # Spawner in Gazebo
        gz_spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', f'/{uav_name}/robot_description',
                '-name', uav_name,
                '-x', str(x_pos),
                '-y', str(y_pos),
                '-z', str(z_pos)
            ],
            output='screen'
        )
        nodes_to_start.append(gz_spawn_entity)
        
        # Static TF: world -> uavN/odom
        tf_broadcaster = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{uav_name}',
            arguments=[
                '--x', str(x_pos),
                '--y', str(y_pos),
                '--z', '0',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'world',
                '--child-frame-id', f'{uav_name}/odom'
            ],
            output='screen'
        )
        nodes_to_start.append(tf_broadcaster)

    return nodes_to_start

def generate_launch_description():
    pkg_uav_gazebo = get_package_share_directory('uav_gazebo')
    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_VERSION', value='harmonic'),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=pkg_uav_gazebo),
        DeclareLaunchArgument(
            'robots',
            default_value='2',
            description='Number of UAVs to spawn'
        ),
        OpaqueFunction(function=launch_setup)
    ])
