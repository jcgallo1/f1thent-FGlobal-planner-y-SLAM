import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('f1tenth_global_path'))
    workspace = package_share.parents[3]
    python_version = f'python{sys.version_info.major}.{sys.version_info.minor}'
    venv_site_packages = workspace / 'venv' / 'lib' / python_version / 'site-packages'
    if not venv_site_packages.is_dir():
        raise RuntimeError(
            f'No se encontro el entorno Python de AutoDRIVE: {venv_site_packages}'
        )

    current_pythonpath = os.environ.get('PYTHONPATH', '')
    bridge_environment = {
        'PYTHONPATH': os.pathsep.join(
            path for path in (str(venv_site_packages), current_pythonpath) if path
        )
    }

    map_yaml = str(package_share / 'maps' / 'f1tenth_map.yaml')
    rviz_config = str(package_share / 'rviz' / 'global_path.rviz')

    return LaunchDescription([
        Node(
            package='autodrive_f1tenth',
            executable='autodrive_incoming_bridge',
            name='autodrive_incoming_bridge',
            output='screen',
            emulate_tty=True,
            additional_env=bridge_environment,
        ),
        Node(
            package='autodrive_f1tenth',
            executable='autodrive_outgoing_bridge',
            name='autodrive_outgoing_bridge',
            output='screen',
            emulate_tty=True,
            additional_env=bridge_environment,
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'frame_id': 'map'}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 0.0,
            }],
        ),
        Node(
            package='f1tenth_global_path',
            executable='path_publisher',
            name='global_path_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_global_path',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
