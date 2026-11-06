import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ur_share = get_package_share_directory('ur_simulation_gz')
    my_share = get_package_share_directory('pickplace_description')

    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_share, 'launch', 'ur_sim_moveit.launch.py')),
        launch_arguments={'ur_type': 'ur5e'}.items())

    spawners = []
    for name in ['table', 'red_cube', 'green_cube', 'overhead_camera']:
        spawners.append(Node(
            package='ros_gz_sim', executable='create',
            arguments=['-file', os.path.join(my_share, 'worlds', name + '.sdf'),
                       '-name', name],
            output='screen'))

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen')

    # Wait 10 s so Gazebo is fully up before spawning anything
    delayed = TimerAction(period=10.0, actions=spawners + [bridge])

    return LaunchDescription([ur_sim, delayed])
