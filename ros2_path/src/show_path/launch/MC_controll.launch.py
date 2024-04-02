import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription()
    MC_Control_node = launch_ros.actions.Node(
        package='show_path',
        namespace='show_path',
        executable='talker',  
        name='control_node'
    )
    ld.add_action( MC_Control_node)

    MC_server_node = launch_ros.actions.Node(
        package='show_path',
        namespace='show_path',
        executable='server',  
        name='MC_server'
    )
    ld.add_action( MC_server_node)

    # Manual_update_node = launch_ros.actions.Node(
    #     package='show_path',
    #     namespace='show_path',
    #     executable='client',  
    #     name='Manual_update_client'
    # )
    # ld.add_action( Manual_update_node)

    rviz = launch_ros.actions.Node(
        package="rviz2",
        executable='rviz2',
        # arguments=['-d',os.path.join(get_package_share_directory("cpp01_launch"),"config","my.rviz")]
        arguments=['-d',os.path.join(get_package_share_directory("show_path"),"config","show_path_rviz.rviz")]
        )
    ld.add_action(rviz) 

    return ld