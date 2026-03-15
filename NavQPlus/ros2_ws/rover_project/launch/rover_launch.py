import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_project')
    default_geojson = os.path.join(pkg_share, 'UCR_Centerlines.json')

    return LaunchDescription([

        DeclareLaunchArgument(
            'geojson_path',
            default_value=default_geojson,
            description='Path to UCR_Centerlines.json'
        ),

        # CAN Bridge (ESP32 sensor data)
        Node(
            package='rover_project',
            executable='can_bridge_node.py',
            name='can_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False}]
        ),

        # UART Node (Arduino motor commands)
        Node(
            package='rover_project',
            executable='uart_node.py',
            name='uart_processor',
            output='screen',
        ),

         # Audio Node ()
        Node(
            package='rover_project',
            executable='audio_input_node.py',
            name='audio_input',
            output='screen',
        ),
        Node(
            package='rover_project',
            executable='audio_output_node.py',
            name='audio_output',
            output='screen',
        ),
        # Fast-SCNN (camera → segmentation + error)
        Node(
            package='rover_project',
            executable='fast_scnn_node.py',
            name='fast_scnn_node',
            output='screen',
            emulate_tty=True,
        ),

        # Path Planner (centerlines → waypoints)
        Node(
            package='rover_project',
            executable='path_planner_node.py',
            name='path_planner',
            output='screen',
            parameters=[{
                'geojson_path': LaunchConfiguration('geojson_path'),
                'walk_paths_only': True,       # True = only Walk_Path=Yes (187 of 766, avoids roads)
                'snap_tolerance_ft': 2.0,      # merge nearby nodes at intersections
                'max_snap_distance_ft': 500.0, # max dist to snap GPS to network
                'replan_deviation_m': 5.0,
                'replan_cooldown_s': 20.0,
                'max_h_acc_m': 10.0,
            }]
        ),

        # Autonomous Drive (safety override: cliff + wall avoidance)
        # Publishes safety/override_active to silence waypoint_follower during corrections.
        Node(
            package='rover_project',
            executable='autonomous_drive_node.py',
            name='autonomous_drive',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'invert_drive': True,          # must match waypoint_follower invert_drive
                'wall_threshold_mm': 500.0,
                'reverse_time_s': 1.0,
                'sensor_time_s': 0.5,
                'maneuver_time_s': 2.0,
                'required_readings': 4,
                'cliff_hold_s': 1.0,
            }]
        ),

        # Waypoint Follower (GPS+IMU → motor commands)
        Node(
            package='rover_project',
            executable='waypoint_follower_node.py',
            name='waypoint_follower',
            output='screen',
            parameters=[{
                'waypoint_reached_m': 3.0,
                'heading_tolerance_deg': 60.0,
                'sharp_turn_deg': 40.0,
                'heading_hysteresis_deg': 10.0,
                'vision_assist_max_heading_err_deg': 35.0,
                'command_rate_hz': 5.0,
                'gps_timeout_s': 5.0,
                'min_fix_type': 2,
                'obstacle_stop_mm': 400,
                'heading_offset_deg': 6.0,
                'magnetic_decline_deg': 13.0,
                'max_h_acc_m': 10.0,
                'invert_drive': True,          # set False if FORWARD/BACKWARD are correct
            }]
        ),

        # Goal sender — listens on /robot_commands and publishes nav/goal
        # Delayed 5s to give path planner time to load GeoJSON
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rover_project',
                    executable='send_goal.py',
                    name='goal_sender',
                    output='screen',
                )
            ]
        ),
        
        # Foxglove Bridge (visualization)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8770,
                'address': '0.0.0.0',
                'send_buffer_limit': 100000

            }]
        ),
    ])