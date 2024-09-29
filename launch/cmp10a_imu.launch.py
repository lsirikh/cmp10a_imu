from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 패키지의 공유 디렉토리 경로를 가져옵니다.
    package_dir = get_package_share_directory('cmp10a_imu')
    # RViz 설정 파일의 전체 경로를 생성합니다.
    rviz_config_path = os.path.join(package_dir, 'rviz', 'imu_visualization.rviz')


    imu_node = Node(
        name='imu_node',
        package='cmp10a_imu',
        executable='cmp10a_imu_node',
        output='screen',
        parameters=[{'port': '/dev/imu_usb'}, {'baud': 9600}],
    )

    imu_visual_node = Node(
        package='cmp10a_imu',
        executable='imu_visual_node',
        name='imu_visualizer',
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path, '-f', 'base_link'],
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )

    return LaunchDescription([
        imu_node,
        imu_visual_node,
        #rviz_node,
        static_tf_node

    ])