# Thesis: Optimising Sensor Fusion Pipelines for Latency and Throughput in Autonomous Driving

**HAW Hamburg — Bachelor's Thesis | Grade: 1.0 (Very Good)**

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue?style=flat)](https://docs.ros.org/en/humble/)
[![Autoware](https://img.shields.io/badge/Autoware-Universe-blue?style=flat)](https://github.com/autowarefoundation/autoware_universe)
[![License](https://img.shields.io/github/license/autowarefoundation/autoware?style=flat&label=License)](LICENSE)

---

## 🎥 Demo Video

[![Watch the demo](https://img.youtube.com/vi/XrM_PXbIQlI/0.jpg)](https://youtu.be/XrM_PXbIQlI)

> LiDAR CenterPoint + YOLOX (TensorRT) + ROI fusion + multi-object tracking on a Waymo Open Dataset rosbag, visualised in RViz2.

---

## Overview

This repository contains the implementation and profiling infrastructure for my Bachelor's thesis. The goal was not to improve individual detection algorithms, but to study how the **integration of modular perception nodes inside ROS 2** affects end-to-end timing — and to identify where time is actually spent in a production-style fusion stack.

The pipeline runs on real-world driving data from the **Waymo Open Dataset**, converted to ROS 2 MCAP bag files, and is instrumented with **ros2_tracing / LTTng** for callback-level performance analysis.

---

## Pipeline Architecture

```
Waymo MCAP bag
    │
    ├── /lidar/concatenated/pointcloud ──► pointcloud_preprocessor
    │                                            │
    │                                     lidar_centerpoint (GPU)
    │                                            │ /objects
    │                                            ▼
    │                               roi_cluster_fusion_node ◄── tensorrt_yolox (GPU)
    │                                            │                  ▲
    └── /camera/front/image ────────────────────────────────────────┘
                                                 │ /perception/fusion/fused_objects
                                                 ▼
                                       multi_object_tracker
                                                 │
                                   /perception/tracking/tracked_objects
```

**Stack:** ROS 2 Humble · Autoware Universe · TensorRT (FP16) · LTTng · Eclipse Trace Compass · rviz2

---

## What I built

- **Perception stack:** composed Autoware Universe modules (CenterPoint 3D, YOLOX-S 2D, ROI fusion, multi-object tracker) into a single multi-threaded component container via custom launch files
- **Dataset pipeline:** converted Waymo Open Dataset `.tfrecord` files to ROS 2 MCAP bags using `waymo2bag` in Docker; verified TF tree and topic alignment in RViz2
- **Tracing infrastructure:** integrated `ros2_tracing` at runtime without modifying Autoware source; post-processed CTF trace files with `tracetools_analysis` and `analyze_latency.py` / `flow_latency.py`
- **Trace Compass analysis:** used the Control Flow view to correlate LiDAR publish → ROI arrival → fusion/tracking bursts on a common timeline across executor threads

---

## Results

### Results at a glance

> **GPU vs CPU split:** `lidar_centerpoint` dominates compute time (P50 13.84 ms), while tracker (2.29 ms) and ROI fusion (0.28 ms) are comparatively small.

> **Key finding:** ~75% of end-to-end latency at P50 is idle waiting for camera/ROI synchronisation, not compute. Processing accounts for only ~25% (16.6 ms out of 66.8 ms).

> **Safety budget:** Pipeline stays within the 100 ms autonomous-driving safety budget at all measured percentiles (P99 = 71 ms).

> One GPU stall outlier at ~211 ms was observed but does not affect P99.

### Callback Duration Distributions

Measured across **n=132 frames** under wall-clock replay at full system load.

| Node | Stage | P50 (ms) | P90 (ms) | P99 (ms) |
|---|---|---|---|---|
| `lidar_centerpoint` | GPU inference (3D detection) | 13.84 | 15.49 | 17.85 |
| `multi_object_tracker` | CPU tracking | 2.29 | 3.34 | 4.18 |
| `roi_cluster_fusion_node` | Projection fusion + publish | 0.28 | — | — |

---

### Throughput

| Node | Realised throughput | Nominal sensor rate |
|---|---|---|
| `lidar_centerpoint` (GPU) | **7.73 Hz** | 10 Hz |
| `multi_object_tracker` (CPU) | **7.79 Hz** | 10 Hz |

The tracker keeps up one-for-one with the fused stream (n=132 callbacks each). The gap to 10 Hz is **not** caused by compute — it is caused by per-frame idle time waiting for the camera/ROI stream.

---

### End-to-End Latency (Detector start → Tracker end)

| Metric | Value |
|---|---|
| **P50** | **67 ms** |
| **P90** | 68 ms |
| **P99** | 71 ms |
| Safety budget (autonomous driving) | 100 ms |

The pipeline stays comfortably within the 100 ms safety budget at all measured percentiles.

---

### Where Time Goes — Latency Breakdown

| Component | Time (ms) | Share |
|---|---|---|
| LiDAR detector callback | 13.64 | 20.4% |
| ROI fusion/publish callback | 0.28 | 0.4% |
| Tracker callback | 2.69 | 4.0% |
| **Processing subtotal** | **16.61** | **24.9%** |
| **Waiting (cross-modal sync)** | **50.22** | **75.1%** |
| **End-to-end (LiDAR → Tracker)** | **66.83** | **100%** |

**Key finding:** ~75% of end-to-end latency at P50 is idle waiting time, not computation. The fusion node subscribes to both LiDAR detections and the camera/ROI stream and emits only when both are available. LiDAR completes in ~14 ms; the matching ROI arrives ~50 ms later. Once both are present, fusion and tracking execute back-to-back in a few milliseconds.

The bottleneck is **cross-modal synchronisation**, not GPU or CPU compute.

---

### Tracing Runs

| Run ID | Trace profile | Bag rate | LiDAR → tracked P50 (ms) | LiDAR → tracked P90 (ms) | LiDAR → tracked P99 (ms) | Throughput (Hz) | Notes |
|---|---|---|---|---|---|---|---|
| baseline-01 | ros | 1.0 | **67** | **68** | **71** | **7.73** | Default executor, no QoS tuning, FP16 TensorRT |

---

## Quick Start

### 1. Run the thesis pipeline

```bash
source install/setup.bash
ros2 launch autoware_launch fusion_perception.launch.py
```

With overrides:

```bash
ros2 launch autoware_launch fusion_perception.launch.py \
  rviz:=true \
  use_sim_time:=true \
  map_path:=/home/hamdy/autoware_map/waymo \
  bag_path:=/home/hamdy/bags/waymo/ros2bags/<your_bag>.db3 \
  bag_rate:=1.0
```

### 2. Tracing profiles

Three LTTng profiles are defined in the launch file:

| Profile | Events captured |
|---|---|
| `ros` | Userspace ROS events (`ros2:*`, `rcl:*`, `rclcpp:*`) |
| `kernel` | Scheduler and IRQ kernel events |
| `ros-kernel` | Both userspace and kernel |

To activate tracing, add one of `trace_ros_only`, `trace_kernel_only`, or `trace_ros_kernel` back into the returned `LaunchDescription` list in `fusion_perception.launch.py`.

### 3. Analyse trace outputs

```bash
python3 tracing/analyze_latency.py ~/tracing/fusion_perception_trace-*
python3 tracing/flow_latency.py ~/tracing/fusion_perception_trace-*
```

Output CSVs:

- `latency_camera_front_image__to__perception_tracking_tracked_objects.csv`
- `latency_lidar_concatenated_pointcloud__to__perception_tracking_tracked_objects.csv`

### 4. Topic flow (default)

| Topic | Description |
|---|---|
| `/objects` | CenterPoint 3D detections |
| `objects_in` → `/objects` | Fusion node input remap |
| `objects_out` → `/perception/fusion/fused_objects` | Fusion node output |
| `/perception/tracking/tracked_objects` | Final tracker output |

---

## Repository structure

```
.
├── src/
│   └── launcher/autoware_launch/   # Custom launch files (fusion_perception.launch.py)
├── tracing/
│   ├── analyze_latency.py          # Callback duration + E2E latency analysis
│   └── flow_latency.py             # Message flow latency analysis
├── centerpoint_params_fast.yaml    # CenterPoint tuned for lower latency
├── centerpoint_params_slow.yaml    # CenterPoint tuned for higher accuracy
└── docker/                         # Docker setup for waymo2bag conversion
```

---

## Limitations

1. Results reflect default QoS and a standard ROS 2 multi-threaded executor — no custom scheduling applied (intentionally out of scope).
2. Bag replay timing can differ from live sensors; the camera/ROI gating effect is visible in both the executor timeline and measured rates.
3. A single 20-second segment (n=132 frames) was profiled; broader coverage may expose additional corner cases.
4. `ros2_tracing` overhead is minimal but nonzero; relative comparisons across nodes within the same session remain valid.

---

## Technologies

`ROS 2 Humble` · `Autoware Universe` · `TensorRT (FP16)` · `YOLOX-S` · `CenterPoint` · `LTTng` · `ros2_tracing` · `Eclipse Trace Compass` · `Waymo Open Dataset` · `MCAP` · `Docker` · `Python`