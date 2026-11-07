import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def spawn(name, my_share, x, y, z, pitch=0.0):
    return Node(
        package='ros_gz_sim', executable='create',
        arguments=['-file', os.path.join(my_share, 'worlds', name + '.sdf'),
                   '-name', name,
                   '-x', str(x), '-y', str(y), '-z', str(z),
                   '-R', '0', '-P', str(pitch), '-Y', '0'],
        output='screen')


def generate_launch_description():
    ur_share = get_package_share_directory('ur_simulation_gz')
    my_share = get_package_share_directory('pickplace_description')

    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_share, 'launch', 'ur_sim_moveit.launch.py')),
        launch_arguments={'ur_type': 'ur5e'}.items())

    load_sensors = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/empty/entity/system/add',
             '--reqtype', 'gz.msgs.EntityPlugin_V',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000',
             '--req',
             'entity {id: 1} plugins {filename: "gz-sim-sensors-system" '
             'name: "gz::sim::systems::Sensors" '
             'innerxml: "<render_engine>ogre2</render_engine>"}'],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen')

    t_sensors = TimerAction(period=10.0, actions=[load_sensors])
    t_objects = TimerAction(period=15.0, actions=[
        spawn('table', my_share, 0.25, 0.0, 0.005),
        spawn('red_cube', my_share, 0.5, 0.1, 0.04),
        spawn('green_cube', my_share, 0.55, -0.1, 0.04)])
    t_camera = TimerAction(period=20.0, actions=[
        spawn('overhead_camera', my_share, 0.55, 0.0, 1.0, 1.5708), bridge])

    return LaunchDescription([ur_sim, t_sensors, t_objects, t_camera])
