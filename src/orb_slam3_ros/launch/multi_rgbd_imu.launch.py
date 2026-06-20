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
            FindPackageShare('orb_slam3_ros'), 'config', 'multi_rgbd_imu.yaml'
        ]),
        description='ROS2 params YAML for the multi-RGB-D-inertial dataset.',
    )
    traj_arg = DeclareLaunchArgument(
        'trajectory_file', default_value='',
        description='If set, save the final trajectory to <trajectory_file>.txt on shutdown.',
    )

    # Front camera (Camera0) of the multi-RGB-D-inertial dataset. camera_info is
    # injected into the prepared bag by tools/prepare_multi_rgbdi.py.
    rgb_topic         = DeclareLaunchArgument('rgb_topic',         default_value='/camera_front/color/image_raw')
    depth_topic       = DeclareLaunchArgument('depth_topic',       default_value='/camera_front/aligned_depth_to_color/image_raw')
    camera_info_topic = DeclareLaunchArgument('camera_info_topic', default_value='/camera_front/camera_info')
    imu_topic         = DeclareLaunchArgument('imu_topic',         default_value='/imu')

    orbslam_node = Node(
        package='orb_slam3_ros',
        executable='ros_node',
        name='orb_slam3',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'vocabulary_file': LaunchConfiguration('vocabulary_file')},
            {'sensor_type': 'rgbd_imu'},
            {'trajectory_file': LaunchConfiguration('trajectory_file')},
        ],
        remappings=[
            ('~/rgb/image_raw',   LaunchConfiguration('rgb_topic')),
            ('~/depth/image_raw', LaunchConfiguration('depth_topic')),
            ('~/rgb/camera_info', LaunchConfiguration('camera_info_topic')),
            ('~/imu',             LaunchConfiguration('imu_topic')),
        ],
    )

    # IMU<->camera extrinsic = pose of cam_front in imu_link = inv(T_CI0) from the
    # dataset calibration. Published via static_transform_publisher (TRANSIENT_LOCAL
    # QoS) because tf baked into a rosbag is VOLATILE and rejected by the tf listener.
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='imu_to_cam_front',
        arguments=['--x', '-0.435392', '--y', '0.022256', '--z', '0.053441',
                   '--qx', '-0.509950', '--qy', '-0.494515', '--qz', '0.485905', '--qw', '0.509080',
                   '--frame-id', 'imu_link', '--child-frame-id', 'cam_front'],
    )

    return LaunchDescription([
        vocab_arg, params_arg, traj_arg,
        rgb_topic, depth_topic, camera_info_topic, imu_topic,
        static_tf, orbslam_node,
    ])
