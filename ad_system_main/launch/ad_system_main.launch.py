from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, LogWarn, DeclareLaunchArgument, LogInfo, SetLaunchConfiguration, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition

MAX_RESTARTS=1

def generate_launch_description():
    # task_arg = DeclareLaunchArgument( #LaunchArgument declaration is required to provide default value for 'task'.
    #     'task',  
    #     default_value='a',
    #     description='Task parameter to set the task'
    # )
    package_names = ["interior_monitoring","lane_detection","localization_pkg","v2x_receiver","environment_model","global_planning","behavior_planning","local_planning","lateral_control","longitudinal_control","v2x_transmitter"]
    node_names = ["interior_monitoring_node","lane_detection_node","localization_node","v2x_receiver_node","environment_node","global_planner","behavior_planner","local_planner","lateral_control","longitudinal_control","v2x_transmitter_node"]
    nodes = []
    handlers = []

    if len(package_names) != len(node_names):
        print("Error: The lengths of package_names and node_names are not the same!")
        sys.exit(1)

    for i in range(len(package_names)):
        restart_counter = f"{package}_restarts"
        SetLaunchConfiguration(restart_counter, "0")
        # Declare a launch configuration to track restarts
        nodes.append(DeclareLaunchArgument(restart_counter, default_value="0"))

        node_action = Node(
            package=package_names[i],
            executable=node_names[i],
            name=node_names[i]
        )

        # Restart if node does not launch successfully
        event_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=node_action,
                on_exit=[
                    LogInfo(msg=f"{package} node exited. Restart attempt: {LaunchConfiguration(restart_counter)}"),
                    SetLaunchConfiguration(restart_counter, PythonExpression([LaunchConfiguration(restart_counter), " + 1"])),
                    IfCondition(
                        PythonExpression([LaunchConfiguration(restart_counter), f" <= {MAX_RESTARTS}"])
                    ).perform(
                        context=None
                    ) and node_action or Shutdown(reason=f"{package} node failed after {MAX_RESTARTS} restart attempt(s). Shutting down.")
                ]
            )
        )

        nodes.append(node_action)
        handlers.append(event_handler)

    LogInfo(msg="All nodes loaded successfully!")

    return LaunchDescription(nodes + handlers)
