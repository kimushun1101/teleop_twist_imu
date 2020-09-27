import launch
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple components."""
    driver = Node(
        node_name='rt_usb_9axisimu_driver',
        package='rt_usb_9axisimu_driver',
        node_executable='rt_usb_9axisimu_driver',
        # parameters=[{
        #     'port': '/dev/ttyUSB0',
        # }]),
        output='screen'
    )

    driver = Node(
        node_name='teleop_twist_imu',
        package='teleop_twist_imu',
        node_executable='teleop_twist_imu',
        # remappings=[('cmd_vel', '/human_vel'),],
        output='screen'
    )
    
    return launch.LaunchDescription([driver])