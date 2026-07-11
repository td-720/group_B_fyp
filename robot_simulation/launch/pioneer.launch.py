import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('robot_simulation')

    # Hardcode the paths for the baked-in robot scenario
    world_path = os.path.join(package_dir, 'worlds', 'world_2.wbt')
    urdf_path = os.path.join(package_dir, 'urdf', 'pioneer_3dx.urdf')
    ros2_control_params = os.path.join(package_dir, 'config', 'pioneer_3dx.yaml')

    # 1. Start Webots in realtime auto-play
    webots = WebotsLauncher(world=world_path, mode='realtime')

    # 2. Start the Driver
    robot_driver = WebotsController(
        robot_name='pioneer3dx', 
        parameters=[
            {'robot_description': urdf_path, 'use_sim_time': True},
            ros2_control_params  
        ]
    )

    # 3. Spawners for the controllers
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller', '--controller-manager', '/controller_manager'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # --- THE FIX: Delay the spawners to allow Webots time to broadcast /clock ---
    delay_joint_state_spawner = TimerAction(
        period=5.0, # Wait 5 seconds
        actions=[joint_state_broadcaster_spawner]
    )

    delay_diffdrive_spawner = TimerAction(
        period=7.0, # Wait 7 seconds (staggered to prevent them from crashing into each other)
        actions=[diffdrive_controller_spawner]
    )

    return LaunchDescription([
        webots,
        robot_driver,
        delay_joint_state_spawner,
        delay_diffdrive_spawner,
        
        # Standard Shutdown Handler
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])