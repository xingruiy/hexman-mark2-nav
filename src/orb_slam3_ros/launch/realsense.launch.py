from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- ORB-SLAM3 arguments ----
    vocab_arg = DeclareLaunchArgument(
        'vocabulary_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('orb_slam3'), 'vocabulary', 'ORBvoc.bin'
        ]),
        description='Path to ORB vocabulary file.',
    )
    camera_config_arg = DeclareLaunchArgument(
        'camera_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('orb_slam3_ros'), 'config', 'd455.yaml'
        ]),
        description='Path to RealSense camera config YAML (d455.yaml or d435i.yaml).',
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('orb_slam3_ros'), 'config', 'd455_slam.yaml'
        ]),
        description='Path to ORB-SLAM3 params YAML (d455_slam.yaml or d455_slam_rgbd.yaml).',
    )
    use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true',
        description='Enable IMU (rgbd_imu) or pure visual (rgbd).',
    )

    # ---- Camera topic remappings ----
    rgb_topic         = DeclareLaunchArgument('rgb_topic',         default_value='/camera/color/image_raw')
    depth_topic       = DeclareLaunchArgument('depth_topic',       default_value='/camera/aligned_depth_to_color/image_raw')
    camera_info_topic = DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info')
    imu_topic         = DeclareLaunchArgument('imu_topic',         default_value='/camera/imu')

    # ---- Optional nodes ----
    run_camera_arg = DeclareLaunchArgument(
        'run_camera', default_value='true',
        description='Launch realsense2_camera_node.')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 with ORB-SLAM3 visualization config.')

    # ---- RealSense camera node ----
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('camera_config_file')],
        condition=IfCondition(LaunchConfiguration('run_camera')),
    )

    # ---- ORB-SLAM3 unified node ----
    sensor_type = PythonExpression([
        "'rgbd_imu' if '", LaunchConfiguration('use_imu'),
        "' == 'true' else 'rgbd'"
    ])
    orbslam_node = Node(
        package='orb_slam3_ros',
        executable='ros_node',
        name='orb_slam3',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'vocabulary_file': LaunchConfiguration('vocabulary_file')},
            {'sensor_type': sensor_type},
        ],
        remappings=[
            ('~/rgb/image_raw',   LaunchConfiguration('rgb_topic')),
            ('~/depth/image_raw', LaunchConfiguration('depth_topic')),
            ('~/rgb/camera_info', LaunchConfiguration('camera_info_topic')),
            ('~/imu',             LaunchConfiguration('imu_topic')),
        ],
    )

    # ---- RViz2 node ----
    rviz_config = PathJoinSubstitution([
        FindPackageShare('orb_slam3_ros'), 'config', 'orb_slam3_d455.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        vocab_arg, camera_config_arg, slam_params_arg, use_imu_arg,
        rgb_topic, depth_topic, camera_info_topic, imu_topic,
        run_camera_arg, use_rviz_arg,
        realsense_node,
        orbslam_node,
        rviz_node,
    ])
