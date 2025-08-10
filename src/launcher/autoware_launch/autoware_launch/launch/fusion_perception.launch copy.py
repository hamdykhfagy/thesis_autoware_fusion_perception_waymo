from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from tracetools_launch.action import Trace


def generate_launch_description():
    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time")
    trace_enabled = LaunchConfiguration("trace_enabled")
    rviz = LaunchConfiguration("rviz")
    data_path = LaunchConfiguration("data_path")
    model_name = LaunchConfiguration("model_name")
    model_path = LaunchConfiguration("model_path")

    # Common dirs
    lidar_dir = PathJoinSubstitution([data_path, "lidar_centerpoint"])
    yolox_dir = PathJoinSubstitution([data_path, "tensorrt_yolox"])
    yolox_param_file = PathJoinSubstitution([
        FindPackageShare("autoware_tensorrt_yolox"),
        "config",
        "yolox_s_plus_opt.param.yaml",
    ])
    # ---- Nodes ----
    lidar_centerpoint = Node(
        package="autoware_lidar_centerpoint",
        executable="autoware_lidar_centerpoint_node",
        name="lidar_centerpoint",
        output="screen",
        remappings=[
            ("~/input/pointcloud", "/lidar/concatenated/pointcloud"),
            ("~/input/pointcloud/cuda", "/lidar/concatenated/pointcloud/cuda"),
            ("~/output/objects", "/objects"),
        ],
        parameters=[
            ParameterFile(PathJoinSubstitution([lidar_dir, "centerpoint_tiny.param.yaml"]), allow_substs=True),
            ParameterFile(PathJoinSubstitution([lidar_dir, "centerpoint_tiny_ml_package.param.yaml"]), allow_substs=True),
            ParameterFile(PathJoinSubstitution([lidar_dir, "detection_class_remapper.param.yaml"]), allow_substs=True),
            ParameterFile(PathJoinSubstitution([lidar_dir, "centerpoint_common.param.yaml"]), allow_substs=True),
            {"use_sim_time": use_sim_time},
            {"build_only": False},
            {"model_path": PathJoinSubstitution([data_path, "lidar_centerpoint"])},
            {"model_name": "centerpoint_tiny"},
        ],
    )

    # ---- YOLOX (standalone) ----
    # Use the executable name you actually have; your logs showed "autoware_tensorrt_yolox_node_exe".
    yolox = Node(
        package="autoware_tensorrt_yolox",
        executable="autoware_tensorrt_yolox_node_exe",
        name="tensorrt_yolox",
        output="screen",
        remappings=[
            ("~/input/image", "/camera/front/image"),
            ("~/output/objects", "/perception/camera/yolox/objects"),
            ("~/output/mask", "/perception/camera/yolox/mask"),
        ],
        parameters=[
            # Inline params (equivalent to XML args)
            {
                "model_path": PathJoinSubstitution([
                    yolox_dir, "yolox-sPlus-opt-pseudoV2-T4-960x960-T4-seg16cls.onnx"
                ]),
                "label_path": PathJoinSubstitution([yolox_dir, "label.txt"]),
                "color_map_path": PathJoinSubstitution([yolox_dir, "semseg_color_map.csv"]),
                "use_decompress": False,
                "build_only": False,
                "use_sim_time": use_sim_time,
            },
            # This mirrors the XML's yolox_param_path being loaded as a param file
            ParameterFile(yolox_param_file, allow_substs=True),
        ],
    )

    rviz2 = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ---- Bag replay (CLI; starts after 10s) ----
    play_bag_proc = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play",
            "/home/hamdy/waymo2bag/rosbag/individual_files_testing_segment-10084636266401282188_1120_000_1140_000_with_camera_labels/testing_bag.db3",  # <-- change to your bag path
            "-l",
        ],
        output="screen",
    )
    delayed_bag = TimerAction(period=15.0, actions=[play_bag_proc])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("trace_enabled", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "data_path",
            default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), "autoware_data"]),
            description="packages data and artifacts directory path",
        ),
        DeclareLaunchArgument("model_name", default_value="centerpoint_tiny",
                              description="options: centerpoint or centerpoint_tiny"),
        DeclareLaunchArgument("model_path",
                              default_value=PathJoinSubstitution([data_path, "lidar_centerpoint"])),

        Trace(session_name="fusion_perception_trace", condition=IfCondition(trace_enabled)),

        # lidar_centerpoint,
        yolox,
        rviz2,
        # delayed_bag,
    ])

