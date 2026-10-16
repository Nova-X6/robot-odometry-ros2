import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('first_project')
    rviz_config = os.path.join(pkg_share, 'rviz', 'first_project.rviz')

    return LaunchDescription([

        # Node 1: compute odometry from /bunker_status
        Node(
            package='first_project',
            executable='odometer',
            name='odometer',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Node 2: compute TF error between GPS (base_link) and ours (base_link2)
        Node(
            package='first_project',
            executable='tf_error',
            name='tf_error',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # RViz: top-down view showing both TFs
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
