# Source: https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_demos/launch/turtlebot4_slam.launch.py
# Requirements:
#   Install Turtlebot3 packages
#   Modify turtlebot3_waffle SDF:
#     1) Edit /opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
#     2) Add
#          <joint name="camera_rgb_optical_joint" type="fixed">
#            <parent>camera_rgb_frame</parent>
#            <child>camera_rgb_optical_frame</child>
#            <pose>0 0 0 -1.57079632679 0 -1.57079632679</pose>
#            <axis>
#              <xyz>0 0 1</xyz>
#            </axis>
#          </joint>
#     3) Rename <link name="camera_rgb_frame"> to <link name="camera_rgb_optical_frame">
#     4) Add <link name="camera_rgb_frame"/>
#     5) Change <sensor name="camera" type="camera"> to <sensor name="camera" type="depth">
#     6) Change image width/height from 1920x1080 to 640x480
#     7) Note that we can increase min scan range from 0.12 to 0.2 to avoid having scans
#        hitting the robot itself
# Example:
#   $ export TURTLEBOT3_MODEL=waffle
#   $ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#
#   SLAM:
#   $ ros2 launch rtabmap_demos turtlebot3_rgbd_sync.launch.py
#   OR
#   $ ros2 launch rtabmap_launch rtabmap.launch.py visual_odometry:=false frame_id:=base_footprint subscribe_scan:=true  approx_sync:=true approx_rgbd_sync:=false odom_topic:=/odom args:="-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1 --Reg/Force3DoF true --Grid/RangeMin 0.2" use_sim_time:=true rgbd_sync:=true rgb_topic:=/camera/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/camera_info qos:=2
#   $ ros2 run topic_tools relay /rtabmap/map /map
#
#   Navigation (install nav2_bringup package):
#     $ ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
#     $ ros2 launch nav2_bringup rviz_launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    qos = LaunchConfiguration("qos")
    localization = LaunchConfiguration("localization")

    icp_parameters = {"odom_frame_id": "icp_odom", "guess_frame_id": "odom", "qos": qos}

    rtabmap_parameters = {
        "subscribe_rgbd": True,
        "subscribe_scan": True,
        "use_action_for_goal": True,
        "odom_sensor_sync": True,
        "qos_scan": qos,
        "qos_image": qos,
        "qos_imu": qos,
        # RTAB-Map's parameters should be strings:
        "Mem/NotLinkedNodesKept": "false",
    }

    # Shared parameters between different nodes
    shared_parameters = {
        "frame_id": "base_footprint",
        "use_sim_time": use_sim_time,
        # RTAB-Map's parameters should be strings:
        "Reg/Strategy": "1",
        "Reg/Force3DoF": "true",
        "Mem/NotLinkedNodesKept": "false",
        "Icp/PointToPlaneMinComplexity": "0.04",  # to be more robust to long corridors with low geometry
    }

    remappings = [
        ("rgb/image", "/limo/depth_camera_link/image_raw"),
        ("rgb/camera_info", "/limo/depth_camera_link/camera_info"),
        ("depth/image", "/limo/depth_camera_link/depth/image_raw"),
        ("odom", "/odometry"),
        ("scan", "/scan"),
    ]

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                choices=["true", "false"],
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "qos", default_value="0", description="QoS used for input sensor topics"
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="false",
                choices=["true", "false"],
                description="Launch rtabmap in localization mode (a map should have been already created).",
            ),
            # Nodes to launch
            Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                output="screen",
                parameters=[
                    {"approx_sync": False, "use_sim_time": use_sim_time, "qos": qos}
                ],
                remappings=remappings,
            ),
            Node(
                package="rtabmap_odom",
                executable="icp_odometry",
                output="screen",
                parameters=[icp_parameters, shared_parameters],
                remappings=remappings,
                arguments=["--ros-args", "--log-level", "icp_odometry:=warn"],
            ),
            # SLAM Mode:
            Node(
                condition=UnlessCondition(localization),
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[rtabmap_parameters, shared_parameters],
                remappings=remappings,
                arguments=["-d"],
            ),
            # Localization mode:
            Node(
                condition=IfCondition(localization),
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[
                    rtabmap_parameters,
                    shared_parameters,
                    {
                        "Mem/IncrementalMemory": "False",
                        "Mem/InitWMWithAllNodes": "True",
                    },
                ],
                remappings=remappings,
            ),
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=[rtabmap_parameters, shared_parameters],
                remappings=remappings,
            ),
        ]
    )
