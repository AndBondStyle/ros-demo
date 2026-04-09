import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('simdemo')
    urdf_model_path = os.path.join(pkg_path, 'config', 'robot.urdf.xacro')
    controllers_yaml_path = os.path.join(pkg_path, 'config', 'controllers.yaml')

    robot_description_config = xacro.process_file(urdf_model_path)
    urdf = robot_description_config.toxml()
    urdf = urdf.replace("CONTROLLER_PARAMS_FILE", controllers_yaml_path)
    robot_description = {'robot_description': urdf}

    sim_time_param = {'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, sim_time_param]
    )

    world_path = os.path.join(get_package_share_directory('simdemo'), 'config', 'world.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        # launch_arguments={'gz_args': '-r empty.sdf'}.items(),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'diff_bot', '-z', '0.5'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/sim/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[sim_time_param]
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller"],
        parameters=[sim_time_param]
    )

    return LaunchDescription([
        gazebo,
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        velocity_controller_spawner,
    ])
