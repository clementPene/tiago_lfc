from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # launch robot simulation in gazebo
    launch_gazebo_tiago = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('tiago_lfc'),
                'launch', 'tiago_gazebo.launch.py'
            ])
        ),
    )
    
    # switch to lfc controllers when controller manager is up
    activate_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('tiago_lfc'),
                'launch', 'switch_to_lfc_controllers.launch.py'
            ])
        ),
    )
    
    # Track whether tuck_arm has been done
    tuck_arm_done = False
    
    def on_tuck_arm_exit_callback(event, context):
        """Callback when tuck_arm.py exits."""
        nonlocal tuck_arm_done
        name = event.process_name or ''
        rc_ok = (event.returncode == 0)
        if 'tuck_arm.py' in name:
            if rc_ok:
                if not tuck_arm_done:
                    tuck_arm_done = True
                    return [
                        LogInfo(msg="tuck_arm.py completed. Activating LFC controllers."),
                        activate_controllers,
                    ]
                else:
                    return [LogInfo(msg="Controllers already activated. Ignoring duplicate tuck_arm exit.")]
            else:
                return [LogInfo(msg="tuck_arm.py failed. Not starting controllers.")]
        return []

    on_tuck_arm_exit_handler = RegisterEventHandler(
        OnProcessExit(on_exit=on_tuck_arm_exit_callback)
    )


    return LaunchDescription([
        on_tuck_arm_exit_handler,
        launch_gazebo_tiago,
    ])