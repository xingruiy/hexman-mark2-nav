from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vocab_arg = DeclareLaunchArgument(
        'vocabulary_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('orb_slam3'), 'vocabulary', 'ORBvoc.bin'
        ]),
        description='Path to ORB vocabulary file.',
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('orb_slam3_ros'), 'config', 'euroc_mono.yaml'
        ]),
        description='ROS2 params YAML for EuRoC monocular-inertial SLAM.',
    )
    traj_arg = DeclareLaunchArgument(
        'trajectory_file', default_value='',
        description='If set, save the final trajectory to <trajectory_file>.txt on shutdown.',
    )

    rgb_topic         = DeclareLaunchArgument('rgb_topic',         default_value='/cam0/image_raw')
    camera_info_topic = DeclareLaunchArgument('camera_info_topic', default_value='/cam0/camera_info')
    imu_topic         = DeclareLaunchArgument('imu_topic',         default_value='/imu0')

    orbslam_node = Node(
        package='orb_slam3_ros',
        executable='ros_node',
        name='orb_slam3',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'vocabulary_file': LaunchConfiguration('vocabulary_file')},
            {'sensor_type': 'monocular_imu'},
            {'trajectory_file': LaunchConfiguration('trajectory_file')},
        ],
        remappings=[
            ('~/rgb/image_raw',   LaunchConfiguration('rgb_topic')),
            ('~/rgb/camera_info', LaunchConfiguration('camera_info_topic')),
            ('~/imu',             LaunchConfiguration('imu_topic')),
        ],
    )

    return LaunchDescription([
        vocab_arg, params_arg, traj_arg,
        rgb_topic, camera_info_topic, imu_topic,
        orbslam_node,
    ])
