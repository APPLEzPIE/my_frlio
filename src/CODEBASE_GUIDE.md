# FR-LIO Core Codebase Guide

This document summarizes the core source tree under `codes/frlio/src` and explains how data flows through the system.

## 1. End-to-End Pipeline

1. Sensor adapters parse raw ROS messages into internal typed data structures.
2. Synchronization module aligns LiDAR with IMU (and optional image) by timestamp.
3. Odometry module performs motion compensation, association, iterative update, and smoothing.
4. Voxel map module stores and queries map points for registration.
5. Optional vision module colorizes LiDAR points and publishes RGB keyframe submaps.

## 2. Directory Responsibilities

### `basic_element/`

- `camera_intrinsic.hpp`: camera projection/inverse projection and border checks.
- `extrinsic_group.hpp`: extrinsic relationship management among LiDAR/IMU/Camera.
- `state_group.hpp`: system state manifold definition and state algebra helpers.

### `format/`

- `lidar_format_processor.hpp`: converts many vendor-specific point types into `PointXYZIRCT`.
- `imu_format_processor.hpp`: IMU value normalization/scaling.
- `image_format_processor.hpp`: resize/undistort/color conversion/equalization.

### `sensor/`

- `sensor_data.hpp`: canonical in-memory data contracts (`Imu_Data`, `Lidar_Data`, `Image_Data`, `Pose_Data`).
- `sensor_synchronizer.h`: timestamp alignment and interpolation for LiDAR-IMU groups.

### `system/`

- `fr_lio.h`: top-level odometry engine and runtime orchestration.
- `imu_integration.h`: IMU propagation and iterated ESKF update.
- `subframe_lio.h`: LiDAR frame splitting and local registration data structure.
- `lidar_motion_compensate_tool.h`: deskew/undistort based on high-frequency poses.
- `associated_pair.hpp`: correspondence representation used by optimization.

### `pointcloud_map/`

- `rc_vox.hpp`: voxel data structure and nearest-neighbor search primitives.
- `rc_vox_map.hpp`: map-level wrapper, insert/search APIs, map parameterization.
- `rc_vox_map.yaml`: map tuning defaults.

### `vision/`

- `rgb_keyframe_generation.hpp`: projects LiDAR submap points to image and builds RGB keyframe submaps.

### `tools/`

- `math_tools.hpp`: SO(3) and related math helpers.
- `time_tools.hpp`: runtime timer and profiling recorder.
- `tools.hpp`: common utilities (trajectory export, geometric helpers, etc.).

## 3. Key Runtime Data Flow

1. `Lidar_Format_Processor::process()` produces standardized point cloud frames.
2. `Sensor_Synchronizer::lidar_imu_sync()` outputs `Lidar_Imu_Measure_Group`.
3. `Odometry` consumes synchronized measurements and updates state.
4. `RC_Vox_Map` is queried/updated during correspondence and map maintenance.
5. If RGB mode enabled, `Rgb_KeyFrame_Generation::add_frame()` emits colored submaps.

## 4. Engineering Conventions

1. Keep point type conversion inside `format/`, not inside optimization logic.
2. Keep synchronization logic in `sensor_synchronizer.h` only.
3. Add new map backends behind `pointcloud_map/` wrappers to avoid touching odometry core.
4. New sensor support should register point struct and add one handler in `lidar_format_processor.hpp`.

## 5. Suggested Extension Entry Points

1. New dataset LiDAR format: extend `Lidar_Format_Processor::LIDAR_TYPE` + handler.
2. New robustness heuristic: add in `fr_lio.h` around frame analysis / data association.
3. New evaluation output: add utility in `tools.hpp` and invoke from odometry pipeline.
