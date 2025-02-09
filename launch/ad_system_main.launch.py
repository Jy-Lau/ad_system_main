from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
import sys


def generate_launch_description():
    # task_arg = DeclareLaunchArgument( #LaunchArgument declaration is required to provide default value for 'task'.
    #     'task',  
    #     default_value='a',
    #     description='Task parameter to set the task'
    # )
    package_names = ["interior_monitoring","lane_detection","localization_pkg","v2x_receiver","environment_model","global_planning","behavior_planning","local_planning","lateral_control","longitudinal_control","v2x_transmitter"]
    node_names = ["interior_monitoring","lane_detection_node","localization_node","v2x_receiver","environment_node","global_planner","behavior_planner","local_planner","lateral_control","longitudinal_control","v2x_transmitter"]

    if len(package_names) != len(node_names):
        print("Error: The lengths of package_names and node_names are not the same!")
        sys.exit(1)
    ld =LaunchDescription()
    for i in range(len(package_names)):

        node_action = Node(
            package=package_names[i],
            executable=node_names[i],
            name=node_names[i],
            output='screen'
        )
        ld.add_action(node_action)

    LogInfo(msg="All nodes loaded successfully!")

    return ld