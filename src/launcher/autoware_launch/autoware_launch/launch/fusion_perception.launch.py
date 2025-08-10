#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Launch: Map component + LiDAR CenterPoint + (optional) YOLOX + ROI Detected-Object Fusion + Multi-Object Tracker + (optional) RViz
# Tracing: LTTng via tracetools_launch with easy on/off + profile selection

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterFile
from tracetools_launch.action import Trace


def generate_launch_description():
    # ---------------------------
    # Launch args (readable refs)
    # ---------------------------
    use_sim_time  = LaunchConfiguration("use_sim_time")
    trace_enabled = LaunchConfiguration("trace_enabled")
    rviz          = LaunchConfiguration("rviz")
    data_path     = LaunchConfiguration("data_path")

    # Common artifact dirs
    yolox_dir       = PathJoinSubstitution([data_path, "tensorrt_yolox"])
    centerpoint_dir = PathJoinSubstitution([data_path, "lidar_centerpoint"])

    # ==========================
    # Map component (PCD + TFs)
    # ==========================
    map_component = GroupAction(actions=[
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("autoware_launch"),
                    "launch",
                    "components",
                    "tier4_map_component.launch.xml",
                ])
            ),
            launch_arguments={
                "map_path": LaunchConfiguration("map_path"),
                "pointcloud_map_file": LaunchConfiguration("pointcloud_map_file"),
                "lanelet2_map_file": LaunchConfiguration("lanelet2_map_file"),
                "use_sim_time": use_sim_time,
            }.items(),
        ),
    ])

    # ============================================================
    # YOLOX (optional): publishes /perception/camera/yolox/objects
    # ============================================================
    yolox_group = GroupAction(actions=[
        # Remap both private (~) and non-~ names so we're safe either way
        SetRemap(src="~/in/image",    dst="/camera/front/image"),
        SetRemap(src="~/out/objects", dst="/perception/camera/yolox/objects"),
        SetRemap(src="~/out/mask",    dst="/perception/camera/yolox/mask"),
        SetRemap(src="in/image",      dst="/camera/front/image"),
        SetRemap(src="out/objects",   dst="/perception/camera/yolox/objects"),
        SetRemap(src="out/mask",      dst="/perception/camera/yolox/mask"),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("autoware_tensorrt_yolox"),
                    "launch",
                    "yolox.launch.xml",
                ])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "model_path": PathJoinSubstitution([
                    yolox_dir, "yolox-sPlus-opt-pseudoV2-T4-960x960-T4-seg16cls.onnx"
                ]),
                "label_path": PathJoinSubstitution([yolox_dir, "label.txt"]),
                "color_map_path": PathJoinSubstitution([yolox_dir, "semseg_color_map.csv"]),
                "yolox_param_path": PathJoinSubstitution([
                    FindPackageShare("autoware_tensorrt_yolox"),
                    "config",
                    "yolox_s_plus_opt.param.yaml",
                ]),
                "use_decompress": TextSubstitution(text="false"),
                "build_only": TextSubstitution(text="false"),
            }.items(),
        ),
    ])

    # ============================================================
    # LiDAR CenterPoint: publishes /objects (DetectedObjects)
    # ============================================================
    centerpoint_group = GroupAction(actions=[
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("autoware_lidar_centerpoint"),
                    "launch",
                    "lidar_centerpoint.launch.xml",
                ])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                # Note: centerpoint YAML expects $(var model_path) and $(var model_name)
                "model_param_path": PathJoinSubstitution([
                    centerpoint_dir, "centerpoint_tiny.param.yaml"
                ]),
                "ml_package_param_path": PathJoinSubstitution([
                    centerpoint_dir, "centerpoint_tiny_ml_package.param.yaml"
                ]),
                "class_remapper_param_path": PathJoinSubstitution([
                    centerpoint_dir, "detection_class_remapper.param.yaml"
                ]),
                "build_only": TextSubstitution(text="false"),
            }.items(),
        ),
    ])

    # ====================================================================
    # ROI Detected-Object Fusion (3D objects + 2D ROIs -> fused objects)
    # ====================================================================
    fusion_params_file = PathJoinSubstitution([
        FindPackageShare("autoware_image_projection_based_fusion"),
        "config",
        "roi_detected_object_fusion.param.yaml",
    ])
    fusion_sync_file = PathJoinSubstitution([
        FindPackageShare("autoware_image_projection_based_fusion"),
        "config",
        "fusion_common.param.yaml",
    ])

    roi_detected_object_fusion = GroupAction(actions=[
        Node(
            package="autoware_image_projection_based_fusion",
            executable="roi_detected_object_fusion_node",
            name="roi_detected_object_fusion",
            output="screen",
            parameters=[
                {"rois_number": LaunchConfiguration("rois_number")},
                ParameterFile(fusion_sync_file, allow_substs=True),
                ParameterFile(fusion_params_file, allow_substs=True),

                # Single-camera (front) inputs by default
                {"input/rois0":        LaunchConfiguration("rois0")},
                {"input/camera_info0": LaunchConfiguration("camera_info0")},
                {"input/image0":       LaunchConfiguration("image0")},
            ],
            remappings=[
                # 3D DetectedObjects in (CenterPoint default is /objects)
                ("input",  LaunchConfiguration("objects_in")),
                # Fused DetectedObjects out
                ("output", LaunchConfiguration("objects_out")),
            ],
        )
    ])

    # ==========================================
    # Multi-Object Tracker (on fused detections)
    # ==========================================
    mot_tracker_yaml = PathJoinSubstitution([
        FindPackageShare("autoware_multi_object_tracker"),
        "config", "multi_object_tracker_node.param.yaml",
    ])
    mot_assoc_yaml = PathJoinSubstitution([
        FindPackageShare("autoware_multi_object_tracker"),
        "config", "data_association_matrix.param.yaml",
    ])
    mot_inputs_yaml = PathJoinSubstitution([
        FindPackageShare("autoware_multi_object_tracker"),
        "config", "input_channels.param.yaml",
    ])

    # Optional override file for topic names/channels
    mot_override_yaml = "/home/hamdy/autoware/src/universe/autoware_universe/perception/autoware_multi_object_tracker/config/mot_override.param.yaml"

    multi_object_tracker = Node(
        package="autoware_multi_object_tracker",
        executable="multi_object_tracker_node",
        name="multi_object_tracker",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            ParameterFile(mot_tracker_yaml, allow_substs=True),
            ParameterFile(mot_assoc_yaml, allow_substs=True),
            ParameterFile(mot_inputs_yaml, allow_substs=True),
            ParameterFile(mot_override_yaml, allow_substs=True),  # optional override
        ],
        remappings=[
            ("output", "/perception/tracking/tracked_objects"),
            # ("input", LaunchConfiguration("objects_out")),  # if not set via YAML override
        ],
    )

    # -------------
    # RViz (optional)
    # -------------
    rviz2 = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # --------------------------------
    # Bag replay (optional, delayed)
    # --------------------------------
    play_bag_proc = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play",
            LaunchConfiguration("bag_path"),
            "-r", LaunchConfiguration("bag_rate"),
        ],
        output="screen",
    )
    delayed_bag = TimerAction(period=15.0, actions=[play_bag_proc])

    # =====================
    # Tracing configuration
    # =====================
    trace_profile = LaunchConfiguration("trace_profile")

    UST_EVENTS_MIN = [
        "ros2:*",
        "rcl:*",
        "rclcpp:*",
    ]

    KERNEL_EVENTS_MIN = [
        "sched_switch",
        "sched_wakeup",
        "sched_waking",
        "irq_handler_entry",
        "irq_handler_exit",
    ]

    # ROS-only (userspace)
    trace_ros_only = Trace(
        condition=IfCondition(PythonExpression(["'", trace_profile, "'", " == 'ros'"])),
        session_name="fusion_perception_trace",
        append_timestamp=LaunchConfiguration("trace_append_ts"),
        base_path=LaunchConfiguration("trace_base_path"),
        events_ust=UST_EVENTS_MIN,
        events_kernel=[],
        context_fields={
            "userspace": ["vpid", "vtid", "procname"],
        },
    )

    # Kernel-only
    trace_kernel_only = Trace(
        condition=IfCondition(PythonExpression(["'", trace_profile, "'", " == 'kernel'"])),
        session_name="fusion_perception_trace",
        append_timestamp=LaunchConfiguration("trace_append_ts"),
        base_path=LaunchConfiguration("trace_base_path"),
        events_ust=[],
        events_kernel=KERNEL_EVENTS_MIN,
        context_fields={
            "kernel": ["pid", "tid", "procname", "prio", "nice"],
        },
    )

    # ROS + Kernel
    trace_ros_kernel = Trace(
        condition=IfCondition(PythonExpression(["'", trace_profile, "'", " == 'ros-kernel'"])),
        session_name="fusion_perception_trace",
        append_timestamp=LaunchConfiguration("trace_append_ts"),
        base_path=LaunchConfiguration("trace_base_path"),
        events_ust=UST_EVENTS_MIN,
        events_kernel=KERNEL_EVENTS_MIN,
        context_fields={
            "userspace": ["vpid", "vtid", "procname"],
            "kernel": ["pid", "tid", "procname", "prio", "nice"],
        },
    )

    # =====================
    # LaunchDescription
    # =====================
    return LaunchDescription([
        # ----------------------
        # Core / common toggles
        # ----------------------
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),

        # ----------------------
        # Tracing control knobs
        # ----------------------
        DeclareLaunchArgument("trace_enabled", default_value="true"),
        DeclareLaunchArgument("trace_profile", default_value="ros"),      # ros | ros-kernel | kernel
        DeclareLaunchArgument("trace_append_ts", default_value="true"),
        DeclareLaunchArgument(
            "trace_base_path",
            default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), "tracing"])
        ),

        # ----------------------
        # Artifact directories
        # ----------------------
        DeclareLaunchArgument(
            "data_path",
            default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), "autoware_data"]),
            description="Root directory containing model artifacts and configs.",
        ),

        # ----------------------------------------------------
        # CenterPoint XML expects $(var model_path/model_name)
        # ----------------------------------------------------
        DeclareLaunchArgument("model_path", default_value=centerpoint_dir),
        DeclareLaunchArgument("model_name", default_value="centerpoint_tiny"),

        # ----------------------
        # Map component args
        # ----------------------
        DeclareLaunchArgument("map_path", default_value="/home/hamdy/autoware_map/waymo"),
        DeclareLaunchArgument("pointcloud_map_file", default_value="map_testing.pcd"),
        DeclareLaunchArgument("lanelet2_map_file", default_value=""),

        # ----------------------
        # ROI fusion wiring
        # ----------------------
        DeclareLaunchArgument("rois_number", default_value="1"),
        DeclareLaunchArgument("objects_in",  default_value="/objects"),
        DeclareLaunchArgument("objects_out", default_value="/perception/fusion/fused_objects"),

        # Single-camera defaults (front). Duplicate for more cameras: rois1/camera_info1/image1, etc.
        DeclareLaunchArgument("rois0",        default_value="/perception/camera/yolox/rois"),
        DeclareLaunchArgument("camera_info0", default_value="/camera/front/camera_info"),
        DeclareLaunchArgument("image0",       default_value="/camera/front/image"),

        # ----------------------
        # MOT config file paths
        # ----------------------
        DeclareLaunchArgument(
            "mot_data_association_matrix_param_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("autoware_launch"),
                "config", "perception", "object_recognition", "tracking",
                "multi_object_tracker", "data_association_matrix.param.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "mot_input_channels_param_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("autoware_launch"),
                "config", "perception", "object_recognition", "tracking",
                "multi_object_tracker", "input_channels.param.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "mot_node_param_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("autoware_launch"),
                "config", "perception", "object_recognition", "tracking",
                "multi_object_tracker", "multi_object_tracker_node.param.yaml",
            ]),
        ),

        # ----------------------
        # Bag options (optional)
        # ----------------------
        DeclareLaunchArgument(
            "bag_path",
            default_value="/home/hamdy/waymo2bag/rosbag/individual_files_testing_segment-10084636266401282188_1120_000_1140_000_with_camera_labels/testing_bag.db3"
        ),
        DeclareLaunchArgument("bag_rate", default_value="1.0"),

        # Start tracing first so early events are captured
        # If you want tracing by default, uncomment one of these:
        # trace_ros_only,
        # trace_kernel_only,
        # trace_ros_kernel,

        # ---- Start things ----
        yolox_group,                 # Optional
        centerpoint_group,
        roi_detected_object_fusion,
        multi_object_tracker,
        rviz2,
        map_component,               # Map first
        delayed_bag,               # comment in to auto-play bag after 15s
    ])
