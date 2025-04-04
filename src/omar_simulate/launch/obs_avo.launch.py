import os

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name = "kasva_simulate"
    pkg_share = get_package_share_directory(pkg_name)
    desc_pkg_name = "kasva_description"
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
        "obstacle_avoidance.sdf"
    )
    
    rviz_cfg_path = os.path.join(
        pkg_share,
        "rviz",
        "obs_avo_cfg.rviz"
    )

    gazebo_cfg_path = os.path.join(
        pkg_share,
        "gazebo",
        "topic_bridge.yaml"
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_node',
        output='screen',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={gazebo_cfg_path}'   
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

    exe_cmd_vel = TimerAction(
        period = 5.1,
        actions = [
            ExecuteProcess(
                cmd='ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once'.split(' '),
                shell=True,
                output='screen'
            )
        ]
    ) 
    
    odom_node = Node(
        package='kasva_sim_scripts',
        executable='odom',
        name='odom_includer',
        output='screen'
    )

    return LaunchDescription([
        set_gz_res_path,
        exe_gazebo,
        exe_rviz,
        spawn_robot,
        bridge_node,
        odom_node,
        exe_cmd_vel
    ])
