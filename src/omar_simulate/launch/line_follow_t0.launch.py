import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = "omar_simulate"
    pkg_share = get_package_share_directory(pkg_name)
    desc_pkg_name = "omar_description"
    desc_pkg_share = get_package_share_directory(desc_pkg_name)
    
    models_path = os.path.join(
        pkg_share,
        "models"
    )

    existing_resource_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    if existing_resource_path:
        new_resource_path = models_path + ":" + existing_resource_path
    else:
        new_resource_path = models_path

    #
    #
    #
    
    set_gz_res_path = SetEnvironmentVariable(
        name = "IGN_GAZEBO_RESOURCE_PATH",
        value = new_resource_path
    )
    
    world_path = os.path.join(
        pkg_share,
        "worlds",
        "line_follow_t0.sdf"
    )
    
    rviz_cfg_path = os.path.join(
        pkg_share,
        "rviz",
        "line_follow_cfg.rviz"
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_node',
        output='screen',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
        ]
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                desc_pkg_share,
                "launch",
                "spawn_gazebo.launch.py"
            )
        )
    )

    exe_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '-v', '4'],
        output='screen'
    )

    exe_rviz = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_cfg_path]
    )

    return LaunchDescription([
        set_gz_res_path,
        exe_gazebo,
        exe_rviz,
        spawn_robot,
        bridge_node
    ])
