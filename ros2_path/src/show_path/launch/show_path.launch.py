import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
# def generate_launch_description():
#     package_name = 'robot_localization'
#     ld = launch.LaunchDescription()
#     pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name) 

#     robot_localization_node = launch_ros.actions.Node(
#         package='robot_localization',
#         executable='ekf_node',
#         name='ekf_filter_node',
#         output='screen',
#         parameters=[os.path.join(pkg_share, 'params/ekf.yaml'), {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}]
#     )

#     ld.add_action(launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
#                                                 description='Flag to enable use_sim_time'))
#     ld.add_action(robot_localization_node) 

#     return ld

def generate_launch_description():
    ld = launch.LaunchDescription()
    filtered_sub_node = launch_ros.actions.Node(
        package='show_path',
        namespace='show_path',
        executable='talker',  
        name='minimal_publisher'
    )
    ld.add_action(filtered_sub_node)
    rviz = launch_ros.actions.Node(
        package="rviz2",
        executable='rviz2',
        # arguments=['-d',os.path.join(get_package_share_directory("cpp01_launch"),"config","my.rviz")]
        arguments=['-d',os.path.join(get_package_share_directory("show_path"),"config","show_path_rviz.rviz")]
        )
    ld.add_action(rviz) 
    return ld