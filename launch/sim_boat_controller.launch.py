from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    pkg_share = FindPackageShare("boat_control")
    gazebo_launch = FindPackageShare("gazebo_ros")

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_launch, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"verbose": "true"}.items(),
    )

    spawn_boat = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_simple_boat",
        output="screen",
        arguments=[
            "-entity", "simple_boat",
            "-file", PathJoinSubstitution([pkg_share, "urdf", "simple_boat.urdf"]),
            "-x", x,
            "-y", y,
            "-z", z,
        ],
    )

    controller = Node(
        package="boat_control",
        executable="boat_controller",
        name="boat_controller_node",
        output="screen",
        parameters=[
            {
                "odom_topic": "/odom",
                "waypoints_topic": "/mission/waypoints",
                "cmd_vel_topic": "/cmd_vel",
                "use_sim_time": use_sim_time,
                "kp_pos": 0.6,
                "ki_pos": 0.02,
                "kp_u": 1.2,
                "ki_u": 0.1,
                "kp_yaw": 1.8,
                "ki_yaw": 0.02,
                "kd_yaw": 0.4,
                "u_max": 1.0,
                "w_max": 1.2,
                "cmd_linear_max": 1.0,
                "cmd_angular_max": 1.2,
                "goal_tolerance": 0.4,
                "slowdown_radius": 2.0,
                "current_vx_hat": 0.0,
                "current_vy_hat": 0.0,
            }
        ],
    )

    waypoints = Node(
        package="boat_control",
        executable="waypoint_publisher",
        name="waypoint_publisher_node",
        output="screen",
        parameters=[
            {
                "topic": "/mission/waypoints",
                "frame_id": "odom",
                "use_sim_time": use_sim_time,
                "waypoints": [
                    5.0, 0.0,
                    5.0, 5.0,
                    0.0, 5.0,
                    0.0, 0.0,
                ],
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.2"),
        world,
        spawn_boat,
        controller,
        waypoints,
    ])
