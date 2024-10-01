from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Launch configuration for IMU parameters
    serial_port = LaunchConfiguration('serial_port', default='/dev/imu_usb')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='9600')
    frame_id = LaunchConfiguration('frame_id', default='imu_link')
    # 패키지의 공유 디렉토리 경로를 가져옵니다.
    package_dir = get_package_share_directory('cmp10a_imu')
    # RViz 설정 파일의 전체 경로를 생성합니다.
    rviz_config_path = os.path.join(package_dir, 'rviz', 'imu_visualization.rviz')
    
    # Transform parameters
    map_to_base_link_transform = ['0', '0', '0', '0', '0', '0', 'map', 'base_link']

    return LaunchDescription([
        # Declare arguments for IMU node
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Port for IMU sensor'
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Baudrate for the IMU sensor'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Frame ID for the IMU sensor'
        ),
        DeclareLaunchArgument(
            'rviz_config_path',
            default_value=rviz_config_path,
            description='Path to the RViz config file'
        ),
        
        # IMU Node
        Node(
            name='imu_node',
            package='cmp10a_imu',
            executable='cmp10a_imu_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
            }],
        ),
        
        # IMU Visualizer Node
        Node(
            package='cmp10a_imu',
            executable='imu_visual_node',
            name='imu_visualizer',
            output='screen'
        ),
        
        # RViz Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path, '-f', 'base_link'],
        ),
        
        # Static Transform Publisher Node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=map_to_base_link_transform,
            output='screen'
        )
    ])