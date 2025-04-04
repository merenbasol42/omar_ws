import os
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

pkg_name = "kasva_description"

def generate_launch_description():
    # Paketin share dizinini al
    pkg_share = get_package_share_directory(pkg_name)
    
    # Xacro dosyasının tam yolunu oluştur
    xacro_file = os.path.join(pkg_share, 'urdf', 'gazebo.urdf.xacro')
    
    # Xacro dosyasını işle (URDF'e çevir)
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()
    
    # İşlenmiş URDF'i geçici bir dosyaya yaz
    tmp_urdf_file = '/tmp/kasva_robot.urdf'
    with open(tmp_urdf_file, 'w') as f:
        f.write(robot_desc)
    
    return LaunchDescription([
        # 1. Ignition Gazebo (Fortress)'u varsayılan world ile başlat
        ExecuteProcess(
            cmd=['ign', 'gazebo', 'empty.sdf', '-v', '4'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # 4. 10 saniye sonra Ignition servisi ile URDF modelini spawn et
        TimerAction(
            period=5.0,  # Gazebo'nun tamamen açılması için bekleme süresi
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ign', 'service', '-s', '/world/empty/create',
                        '--reqtype', 'ignition.msgs.EntityFactory',
                        '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '1000',
                        '--req', f'sdf_filename: "{tmp_urdf_file}", name: "robo_kasva"'
                    ],
                    output='screen'
                )
            ]
        )
    ])
