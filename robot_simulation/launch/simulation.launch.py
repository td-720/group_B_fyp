# import os
# import launch
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction, EmitEvent, RegisterEventHandler
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from webots_ros2_driver.webots_launcher import WebotsLauncher
# from webots_ros2_driver.webots_controller import WebotsController
# # from webots_ros2_driver.urdf_spawner import URDFSpawner
# from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher
# from launch.events import Shutdown
# from launch.event_handlers import OnProcessExit
# import datetime

# def launch_setup(context, *args, **kwargs):
#     package_dir = get_package_share_directory('robot_simulation')

#     world_arg = LaunchConfiguration('world').perform(context)
#     robot_name_str = LaunchConfiguration('robot_name').perform(context)
#     scenario_name_str = LaunchConfiguration('scenario').perform(context)
    
#     # --- THE DEVELOPER BACKDOOR: Argument Capture ---
#     urdf_arg = LaunchConfiguration('urdf').perform(context)
#     yaml_arg = LaunchConfiguration('yaml').perform(context)

#     # --- DYNAMIC ROUTING LOGIC ---
#     # 1. World
#     if os.path.isabs(world_arg):
#         world_path = world_arg
#     else:
#         world_path = os.path.join(package_dir, 'worlds', world_arg)

#     # 2. URDF (Hardware) - Defaults to package if GUI calls it, uses absolute path if terminal overrides
#     if os.path.isabs(urdf_arg):
#         robot_description_path = urdf_arg
#     else:
#         robot_description_path = os.path.join(package_dir, 'urdf', urdf_arg)

#     # 3. YAML (Control Params)
#     if os.path.isabs(yaml_arg):
#         ros2_control_params = yaml_arg
#     else:
#         ros2_control_params = os.path.join(package_dir, 'config', yaml_arg)

#     # Boot the Webots Environment
#     webots = WebotsLauncher(
#         world=world_path,
#         mode='realtime'
#     )

#     # Boot the ROS 2 Driver (The Nervous System)
#     robot_driver = WebotsController(
#         robot_name=robot_name_str,
#         parameters=[
#             {'robot_description': robot_description_path, 'use_sim_time': True},
#             ros2_control_params  
#         ]
#     )

#     # spawn_robot = URDFSpawner(
#     #     name=robot_name_str,
#     #     urdf_path=robot_description_path, # Passed from the GUI or default
#     #     translation='0 0 0.1',
#     #     rotation='0 0 1 0'
#     # )

#     ros2_supervisor = Ros2SupervisorLauncher()

#     diffdrive_controller_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         output='screen',
#         arguments=['diffdrive_controller', '--controller-manager', '/controller_manager'],
#     )

#     joint_state_broadcaster_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         output='screen',
#         arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
#     )

#     bag_name = 'obstacle_run_' + datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
#     rosbag_record = ExecuteProcess(
#         cmd=['ros2', 'bag', 'record', '-o', bag_name, '--use-sim-time',
#              '/scan', '/diffdrive_controller/cmd_vel_unstamped', '/diffdrive_controller/odom'],
#         output='screen'
#     )
    
#     delayed_controller_spawners = TimerAction(
#         period=10.0,
#         actions=[
#             diffdrive_controller_spawner,
#             joint_state_broadcaster_spawner,
#             rosbag_record
#         ]
#     )

#     with open(robot_description_path, 'r') as f:
#         robot_urdf_content = f.read()

#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'robot_description': robot_urdf_content,
#             'use_sim_time': True 
#         }]
#     )

#     # --- THE ALGORITHM FORK (SANDBOX EXECUTION) ---
#     if os.path.isabs(scenario_name_str) and scenario_name_str.endswith('.py'):
#         print(f"\n[AFIT GCS] SANDBOX OVERRIDE: Executing raw Python script -> {scenario_name_str}\n")
#         # Bypass colcon. Run the user's script directly.
#         scenario_brain = ExecuteProcess(
#             cmd=['python3', scenario_name_str],
#             output='screen',
#         )
#     else:
#         print(f"\n[AFIT GCS] STANDARD SCENARIO: Executing internal node -> {scenario_name_str}\n")
        
#         # Determine if we need an interactive terminal
#         # This fixes the termios.error: (25, 'Inappropriate ioctl for device') crash
#         if scenario_name_str == 'teleop_node':
#             scenario_brain = Node(
#                 package='robot_simulation',
#                 executable=scenario_name_str,
#                 output='screen',
#                 parameters=[{'use_sim_time': True}],
#                 emulate_tty=True,
#                 prefix='xterm -e' # Forces a new terminal window to open for keyboard input
#             )
#         else:
#             # Standard execution from the compiled workspace for autonomous nodes
#             scenario_brain = Node(
#                 package='robot_simulation',
#                 executable=scenario_name_str,
#                 output='screen',
#                 parameters=[{'use_sim_time': True}],
#             )

#     shutdown_handler = RegisterEventHandler(
#         event_handler=OnProcessExit(
#             target_action=scenario_brain, 
#             on_exit=[EmitEvent(event=Shutdown())] 
#         )
#     )

#     return [
#         webots,
#         ros2_supervisor,
#         robot_driver,
#         delayed_controller_spawners,
#         scenario_brain,
#         # spawn_robot,
#         robot_state_publisher,
#         shutdown_handler,
#         launch.actions.RegisterEventHandler(
#             event_handler=launch.event_handlers.OnProcessExit(
#                 target_action=webots,
#                 on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
#             )
#         )
#     ]

# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument('world', default_value='obstacle_course.wbt'),
#         DeclareLaunchArgument('robot_name', default_value='robot_a'),
#         DeclareLaunchArgument('scenario', default_value='teleop_node'), 
        
#         DeclareLaunchArgument('urdf', default_value='robot_a.urdf'),
#         DeclareLaunchArgument('yaml', default_value='ros2_control.yaml'),
        
#         OpaqueFunction(function=launch_setup)
#     ])





















#ekf added
# import os
# import launch
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction, EmitEvent, RegisterEventHandler
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from webots_ros2_driver.webots_launcher import WebotsLauncher
# from webots_ros2_driver.webots_controller import WebotsController
# from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher
# from launch.events import Shutdown
# from launch.event_handlers import OnProcessExit
# import datetime

# def launch_setup(context, *args, **kwargs):
#     package_dir = get_package_share_directory('robot_simulation')

#     world_arg = LaunchConfiguration('world').perform(context)
#     robot_name_str = LaunchConfiguration('robot_name').perform(context)
#     scenario_name_str = LaunchConfiguration('scenario').perform(context)
    
#     urdf_arg = LaunchConfiguration('urdf').perform(context)
#     yaml_arg = LaunchConfiguration('yaml').perform(context)

#     if os.path.isabs(world_arg):
#         world_path = world_arg
#     else:
#         world_path = os.path.join(package_dir, 'worlds', world_arg)

#     if os.path.isabs(urdf_arg):
#         robot_description_path = urdf_arg
#     else:
#         robot_description_path = os.path.join(package_dir, 'urdf', urdf_arg)

#     if os.path.isabs(yaml_arg):
#         ros2_control_params = yaml_arg
#     else:
#         ros2_control_params = os.path.join(package_dir, 'config', yaml_arg)

#     # [ADDED] EKF Config Path
#     ekf_config_path = os.path.join(package_dir, 'config', 'ekf.yaml')

#     webots = WebotsLauncher(
#         world=world_path,
#         mode='realtime'
#     )

#     robot_driver = WebotsController(
#         robot_name=robot_name_str,
#         parameters=[
#             {'robot_description': robot_description_path, 'use_sim_time': True},
#             ros2_control_params  
#         ]
#     )

#     ros2_supervisor = Ros2SupervisorLauncher()

#     diffdrive_controller_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         output='screen',
#         arguments=['diffdrive_controller', '--controller-manager', '/controller_manager'],
#     )

#     joint_state_broadcaster_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         output='screen',
#         arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
#     )

#     # [FIXED] Bag logging includes EKF output and IMU
#     bag_name = 'obstacle_run_' + datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
#     rosbag_record = ExecuteProcess(
#         cmd=['ros2', 'bag', 'record', '-o', bag_name, '--use-sim-time',
#              '/scan', '/diffdrive_controller/cmd_vel_unstamped', 
#              '/diffdrive_controller/odom', '/imu/data', '/odometry/filtered'],
#         output='screen'
#     )
    
#     # [ADDED] The EKF Node
#     ekf_node = Node(
#         package='robot_localization',
#         executable='ekf_node',
#         name='ekf_filter_node',
#         output='screen',
#         parameters=[
#             ekf_config_path, 
#             {'use_sim_time': True} # CRITICAL
#         ],
#         remappings=[
#             ('/odom', '/diffdrive_controller/odom')
#         ]
#     )

#     delayed_controller_spawners = TimerAction(
#         period=10.0,
#         actions=[
#             diffdrive_controller_spawner,
#             joint_state_broadcaster_spawner,
#             rosbag_record,
#             ekf_node  # Boot EKF
#         ]
#     )

#     with open(robot_description_path, 'r') as f:
#         robot_urdf_content = f.read()

#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'robot_description': robot_urdf_content,
#             'use_sim_time': True 
#         }]
#     )

#     if os.path.isabs(scenario_name_str) and scenario_name_str.endswith('.py'):
#         print(f"\n[AFIT GCS] SANDBOX OVERRIDE: Executing raw Python script -> {scenario_name_str}\n")
#         scenario_brain = ExecuteProcess(
#             cmd=['python3', scenario_name_str],
#             output='screen',
#         )
#     else:
#         print(f"\n[AFIT GCS] STANDARD SCENARIO: Executing internal node -> {scenario_name_str}\n")
#         if scenario_name_str == 'teleop_node':
#             scenario_brain = Node(
#                 package='robot_simulation',
#                 executable=scenario_name_str,
#                 output='screen',
#                 parameters=[{'use_sim_time': True}],
#                 emulate_tty=True,
#                 prefix='xterm -e' 
#             )
#         else:
#             scenario_brain = Node(
#                 package='robot_simulation',
#                 executable=scenario_name_str,
#                 output='screen',
#                 parameters=[{'use_sim_time': True}],
#             )

#     shutdown_handler = RegisterEventHandler(
#         event_handler=OnProcessExit(
#             target_action=scenario_brain, 
#             on_exit=[EmitEvent(event=Shutdown())] 
#         )
#     )

#     return [
#         webots,
#         ros2_supervisor,
#         robot_driver,
#         delayed_controller_spawners,
#         scenario_brain,
#         robot_state_publisher,
#         shutdown_handler,
#         launch.actions.RegisterEventHandler(
#             event_handler=launch.event_handlers.OnProcessExit(
#                 target_action=webots,
#                 on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
#             )
#         )
#     ]

# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument('world', default_value='obstacle_course.wbt'),
#         DeclareLaunchArgument('robot_name', default_value='robot_a'),
#         DeclareLaunchArgument('scenario', default_value='teleop_node'), 
#         DeclareLaunchArgument('urdf', default_value='robot_a.urdf'),
#         DeclareLaunchArgument('yaml', default_value='ros2_control.yaml'),
#         OpaqueFunction(function=launch_setup)
#     ])
























import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
import datetime

def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('robot_simulation')

    world_arg = LaunchConfiguration('world').perform(context)
    robot_name_str = LaunchConfiguration('robot_name').perform(context)
    scenario_name_str = LaunchConfiguration('scenario').perform(context)
    
    urdf_arg = LaunchConfiguration('urdf').perform(context)
    yaml_arg = LaunchConfiguration('yaml').perform(context)

    if os.path.isabs(world_arg):
        world_path = world_arg
    else:
        world_path = os.path.join(package_dir, 'worlds', world_arg)

    if os.path.isabs(urdf_arg):
        robot_description_path = urdf_arg
    else:
        robot_description_path = os.path.join(package_dir, 'urdf', urdf_arg)

    if os.path.isabs(yaml_arg):
        ros2_control_params = yaml_arg
    else:
        ros2_control_params = os.path.join(package_dir, 'config', yaml_arg)

    # [ADDED] EKF Config Path
    ekf_config_path = os.path.join(package_dir, 'config', 'ekf.yaml')

    webots = WebotsLauncher(
        world=world_path,
        mode='realtime'
    )

    robot_driver = WebotsController(
        robot_name=robot_name_str,
        parameters=[
            {'robot_description': robot_description_path, 'use_sim_time': True},
            ros2_control_params  
        ]
    )

    ros2_supervisor = Ros2SupervisorLauncher()

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

    # [FIXED] Bag logging includes EKF output and IMU
    bag_name = 'obstacle_run_' + datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_name, '--use-sim-time',
             '/scan', '/diffdrive_controller/cmd_vel_unstamped', 
             '/diffdrive_controller/odom', '/imu/data', '/odometry/filtered'],
        output='screen'
    )
    
    # [ADDED] The EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path, 
            {'use_sim_time': True} # CRITICAL
        ],
        remappings=[
            ('/odom', '/diffdrive_controller/odom')
        ]
    )

    delayed_controller_spawners = TimerAction(
        period=10.0,
        actions=[
            diffdrive_controller_spawner,
            joint_state_broadcaster_spawner,
            rosbag_record,
            ekf_node  # Boot EKF
        ]
    )

    with open(robot_description_path, 'r') as f:
        robot_urdf_content = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_urdf_content,
            'use_sim_time': True 
        }]
    )

    if os.path.isabs(scenario_name_str) and scenario_name_str.endswith('.py'):
        print(f"\n[AFIT GCS] SANDBOX OVERRIDE: Executing raw Python script -> {scenario_name_str}\n")
        scenario_brain = ExecuteProcess(
            cmd=['python3', scenario_name_str],
            output='screen',
        )
    else:
        print(f"\n[AFIT GCS] STANDARD SCENARIO: Executing internal node -> {scenario_name_str}\n")
        if scenario_name_str == 'teleop_node':
            scenario_brain = Node(
                package='robot_simulation',
                executable=scenario_name_str,
                output='screen',
                parameters=[{'use_sim_time': True}],
                emulate_tty=True,
                prefix='xterm -e' 
            )
        else:
            scenario_brain = Node(
                package='robot_simulation',
                executable=scenario_name_str,
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    # THE FIX: This pipes the CLI/GUI argument down into the node's parameters
                    'relative_waypoints': LaunchConfiguration('relative_waypoints')
                }],
            )

    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=scenario_brain, 
            on_exit=[EmitEvent(event=Shutdown())] 
        )
    )

    return [
        webots,
        ros2_supervisor,
        robot_driver,
        delayed_controller_spawners,
        scenario_brain,
        robot_state_publisher,
        shutdown_handler,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='obstacle_course.wbt'),
        DeclareLaunchArgument('robot_name', default_value='robot_a'),
        DeclareLaunchArgument('scenario', default_value='teleop_node'), 
        DeclareLaunchArgument('urdf', default_value='robot_a.urdf'),
        DeclareLaunchArgument('yaml', default_value='ros2_control.yaml'),
        
        # THE FIX: This exposes the argument so the launch file doesn't ignore it
        DeclareLaunchArgument('relative_waypoints', default_value='[5.0, 0.0, 10.0, 0.0, 15.0, 0.0]'),
        OpaqueFunction(function=launch_setup)
    ])











# mods for tests
# import os
# import launch
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction, EmitEvent, RegisterEventHandler
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from webots_ros2_driver.webots_launcher import WebotsLauncher
# from webots_ros2_driver.webots_controller import WebotsController
# from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher
# from launch.events import Shutdown
# from launch.event_handlers import OnProcessExit
# import datetime

# def launch_setup(context, *args, **kwargs):
#     package_dir = get_package_share_directory('robot_simulation')

#     world_arg = LaunchConfiguration('world').perform(context)
#     robot_name_str = LaunchConfiguration('robot_name').perform(context)
#     scenario_name_str = LaunchConfiguration('scenario').perform(context)
    
#     urdf_arg = LaunchConfiguration('urdf').perform(context)
#     yaml_arg = LaunchConfiguration('yaml').perform(context)

#     if os.path.isabs(world_arg):
#         world_path = world_arg
#     else:
#         world_path = os.path.join(package_dir, 'worlds', world_arg)

#     if os.path.isabs(urdf_arg):
#         robot_description_path = urdf_arg
#     else:
#         robot_description_path = os.path.join(package_dir, 'urdf', urdf_arg)

#     if os.path.isabs(yaml_arg):
#         ros2_control_params = yaml_arg
#     else:
#         ros2_control_params = os.path.join(package_dir, 'config', yaml_arg)

#     # [ADDED] EKF Config Path
#     ekf_config_path = os.path.join(package_dir, 'config', 'ekf.yaml')

#     webots = WebotsLauncher(
#         world=world_path,
#         mode='realtime'
#     )

#     robot_driver = WebotsController(
#         robot_name=robot_name_str,
#         parameters=[
#             {'robot_description': robot_description_path, 'use_sim_time': True},
#             ros2_control_params  
#         ]
#     )

#     ros2_supervisor = Ros2SupervisorLauncher()

#     diffdrive_controller_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         output='screen',
#         arguments=['diffdrive_controller', '--controller-manager', '/controller_manager'],
#     )

#     joint_state_broadcaster_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         output='screen',
#         arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
#     )

#     # [FIXED] Bag logging includes EKF output and IMU
#     bag_name = 'obstacle_run_' + datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
#     rosbag_record = ExecuteProcess(
#         cmd=['ros2', 'bag', 'record', '-o', bag_name, '--use-sim-time',
#              '/scan', '/diffdrive_controller/cmd_vel_unstamped', 
#              '/diffdrive_controller/odom', '/imu/data', '/odometry/filtered'],
#         output='screen'
#     )
    
#     # [ADDED] The EKF Node
#     ekf_node = Node(
#         package='robot_localization',
#         executable='ekf_node',
#         name='ekf_filter_node',
#         output='screen',
#         parameters=[
#             ekf_config_path, 
#             {'use_sim_time': True} # CRITICAL
#         ],
#         remappings=[
#             ('/odom', '/diffdrive_controller/odom')
#         ]
#     )

#     delayed_controller_spawners = TimerAction(
#         period=10.0,
#         actions=[
#             diffdrive_controller_spawner,
#             joint_state_broadcaster_spawner,
#             rosbag_record,
#             ekf_node  # Boot EKF
#         ]
#     )

#     with open(robot_description_path, 'r') as f:
#         robot_urdf_content = f.read()

#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'robot_description': robot_urdf_content,
#             'use_sim_time': True 
#         }]
#     )

#     # --- BASE LAUNCH ACTIONS (Always execute these) ---
#     launch_actions = [
#         webots,
#         ros2_supervisor,
#         robot_driver,
#         delayed_controller_spawners,
#         robot_state_publisher,
#         launch.actions.RegisterEventHandler(
#             event_handler=launch.event_handlers.OnProcessExit(
#                 target_action=webots,
#                 on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
#             )
#         )
#     ]

#     # --- CONDITIONAL SCENARIO EXECUTION ---
#     # Only execute a brain node if the user or GUI explicitly requests one
#     if scenario_name_str.lower() != 'none' and scenario_name_str.strip() != '':
        
#         if os.path.isabs(scenario_name_str) and scenario_name_str.endswith('.py'):
#             print(f"\n[AFIT GCS] SANDBOX OVERRIDE: Executing raw Python script -> {scenario_name_str}\n")
#             scenario_brain = ExecuteProcess(
#                 cmd=['python3', scenario_name_str],
#                 output='screen',
#             )
#         else:
#             print(f"\n[AFIT GCS] STANDARD SCENARIO: Executing internal node -> {scenario_name_str}\n")
#             if scenario_name_str == 'teleop_node':
#                 scenario_brain = Node(
#                     package='robot_simulation',
#                     executable=scenario_name_str,
#                     output='screen',
#                     parameters=[{'use_sim_time': True}],
#                     emulate_tty=True,
#                     prefix='xterm -e' 
#                 )
#             else:
#                 scenario_brain = Node(
#                     package='robot_simulation',
#                     executable=scenario_name_str,
#                     output='screen',
#                     parameters=[{
#                         'use_sim_time': True,
#                         # THE FIX: This pipes the CLI/GUI argument down into the node's parameters
#                         'relative_waypoints': LaunchConfiguration('relative_waypoints')
#                     }],
#                 )

#         # Only register the scenario shutdown handler if a scenario is actually running
#         shutdown_handler = RegisterEventHandler(
#             event_handler=OnProcessExit(
#                 target_action=scenario_brain, 
#                 on_exit=[EmitEvent(event=Shutdown())] 
#             )
#         )

#         # Append the conditional nodes to the final execution list
#         launch_actions.extend([scenario_brain, shutdown_handler])
#     else:
#         print("\n[AFIT GCS] ENVIRONMENT ONLY: No scenario requested. Booting simulation and awaiting external commands.\n")

#     return launch_actions

# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument('world', default_value='obstacle_course.wbt'),
#         DeclareLaunchArgument('robot_name', default_value='robot_a'),
#         # CHANGED DEFAULT VALUE TO 'none' TO ALLOW GUI CONTROL
#         DeclareLaunchArgument('scenario', default_value='none', description='Scenario node/script to run. Use "none" to just launch the environment.'), 
#         DeclareLaunchArgument('urdf', default_value='robot_a.urdf'),
#         DeclareLaunchArgument('yaml', default_value='ros2_control.yaml'),
        
#         # THE FIX: This exposes the argument so the launch file doesn't ignore it
#         DeclareLaunchArgument('relative_waypoints', default_value='[1.0, 0.0, 2.0, 2.0, 3.0, 0.0, 4.0, 2.0]'),
        
#         OpaqueFunction(function=launch_setup)
#     ])
