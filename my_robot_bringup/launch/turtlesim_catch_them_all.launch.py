from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim_node",
            output="screen",
            emulate_tty=True
        ),
        Node(
            package="turtlesim_catch_them_all",
            executable="turtle_spawner_test",
            name="turtle_spawner",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"spawn_rate": 0.5},
                {"prefix_name": "turtle"}
            ]
        ),
        Node(
            package="turtlesim_catch_them_all",
            executable="turtle_controller",
            name="turtle_controller",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"catch_closes_turtle_first": True}
            ]
        )
    ])