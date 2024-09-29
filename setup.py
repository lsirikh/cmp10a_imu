from setuptools import setup, find_packages

package_name = 'cmp10a_imu'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cmp10a_imu.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/imu_visualization.rviz']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='lsirikh@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmp10a_imu_node = cmp10a_imu.cmp10a_imu_node:main',
            'imu_visual_node = cmp10a_imu.imu_visual_node:main'
        ],
    },
)
