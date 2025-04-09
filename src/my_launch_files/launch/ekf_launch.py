from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Command 1: Run reset_1
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'cylin_world.launch.py'],
            name='world_launch'
        ),
        # Command 2: Run reset_2
        TimerAction(
            period=3.0,  # Adjust delay if necessary
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'ekf', 'mapToCoords'],
                    name='map2coordinate'
                )
            ]
        ),
        # Command 3: Run micro_ros_agent with /dev/ttyUSB0
        TimerAction(
            period=4.0,  # Adjust delay if necessary
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'ekf', 'localization'],
                    name='ekf_runner'
                )
            ]
        ),
        # Command 4: Run micro_ros_agent with /dev/ttyUSB1
        TimerAction(
            period=3.0,  # Adjust delay if necessary
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'rviz2', 'rviz2'],
                    name='viz'
                )
            ]
        ),
        
        TimerAction(
            period=6.0,  # Adjust delay if necessary
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'cpp_pubsub', 'autonode'],
                    name='viz'
                )
            ]
        ),
    ])

