#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <unordered_map>
#include <iostream>

#include <pcl/impl/pcl_base.hpp>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>

/**
 * @brief IMU单帧测量。
 * timestamp_ 采用秒为单位，acc_/gyr_均在IMU坐标系下。
 */
struct Imu_Data {
    Imu_Data(double timestamp, double acc_x, double acc_y, double acc_z, double gyr_x, double gyr_y, double gyr_z)
        : timestamp_(timestamp)
        , acc_(acc_x, acc_y, acc_z)
        , gyr_(gyr_x, gyr_y, gyr_z)
    {
    }

    Imu_Data(double timestamp, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
        : timestamp_(timestamp)
        , acc_(acc)
        , gyr_(gyr)
    {
    }

    double timestamp_;
    Eigen::Vector3d acc_;
    Eigen::Vector3d gyr_;
};

/**
 * @brief LiDAR帧数据（已格式化）。
 * begin_time_/end_time_ 描述该帧扫描时间范围。
 */
template <typename PointType> struct Lidar_Data {
    Lidar_Data() = default;
    Lidar_Data(double begin_time, double end_time, typename pcl::PointCloud<PointType>::Ptr cloud)
        : begin_time_(begin_time)
        , end_time_(end_time)
        , cloud_(cloud)
    {
    }

    double begin_time_;
    double end_time_;
    typename pcl::PointCloud<PointType>::Ptr cloud_;
};

/**
 * @brief 原始LiDAR消息封装（未做点类型转换）。
 */
struct Raw_Lidar_Data {
    Raw_Lidar_Data(double timestamp, pcl::PCLPointCloud2::Ptr cloud)
        : timestamp_(timestamp)
        , cloud_(cloud)
    {
    }

    double timestamp_;
    pcl::PCLPointCloud2::Ptr cloud_;
};

/**
 * @brief 图像帧数据。
 */
struct Image_Data {
    Image_Data(double timestamp, cv::Mat &img)
        : timestamp_(timestamp)
        , img_(img)
    {
    }

    double timestamp_;
    cv::Mat img_;
};

/**
 * @brief 位姿数据。
 * quat_/pos_ 表示某一时刻传感器在参考坐标系中的姿态与位置。
 */
struct Pose_Data {
    Pose_Data(double timestamp, const Eigen::Quaterniond &quat, const Eigen::Vector3d &pos)
        : timestamp_(timestamp)
        , quat_(quat)
        , pos_(pos)
    {
    }

    Pose_Data(double timestamp, const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos)
        : timestamp_(timestamp)
        , quat_(rot)
        , pos_(pos)
    {
    }

    double timestamp_;
    Eigen::Quaterniond quat_;
    Eigen::Vector3d pos_;
};