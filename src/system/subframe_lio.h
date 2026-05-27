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

#include "./imu_integration.h"

/**
 * @brief LiDAR子帧数据结构。
 *
 * 一个Sub_Frame对应一次LiDAR帧及其时间范围内IMU测量，
 * 用于退化检测、点云关联和局部优化过程中的中间状态承载。
 */
template <typename PointType> class Sub_Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Sub_Frame() = default;
    Sub_Frame(std::shared_ptr<Lidar_Data<PointType>> lidar_data, const std::deque<std::shared_ptr<Imu_Data>> &imu_datas)
        : lidar_data_(lidar_data)
        , imu_datas_(imu_datas)
        , timestamp_(lidar_data->end_time_)
        , flg_degrad_(true)
        , flg_has_estimated_(false)
        , smooth_count_(0)
    {
        const_speed_eskf_.reset(new Imu_Integration(imu_datas_, false));
    }

    std::shared_ptr<Imu_Integration> get_const_speed_eskf()
    {
        return const_speed_eskf_;
    }

    std::shared_ptr<Lidar_Data<PointType>> get_lidar_data()
    {
        return lidar_data_;
    }

    typename pcl::PointCloud<PointType>::Ptr get_local_undistorted_points()
    {
        return local_undistorted_points_;
    }

    void set_local_undistorted_points(typename pcl::PointCloud<PointType>::Ptr local_undistorted_points)
    {
        local_undistorted_points_ = local_undistorted_points;
    }

    std::shared_ptr<Image_Data> get_image_data()
    {
        return image_data_;
    }

    void set_image_data(std::shared_ptr<Image_Data> image_data)
    {
        image_data_ = image_data;
    }

    double get_timestamp()
    {
        return timestamp_;
    }

    // debug
    bool flg_degrad_;
    bool flg_has_estimated_;
    bool flg_use_for_registration_;
    unsigned long long id_;
    int smooth_count_;

    std::vector<std::pair<bool, PointType>> closest_map_pts;

private:
    std::shared_ptr<Imu_Integration> const_speed_eskf_;
    std::shared_ptr<Lidar_Data<PointType>> lidar_data_;
    std::deque<std::shared_ptr<Imu_Data>> imu_datas_;
    typename pcl::PointCloud<PointType>::Ptr local_undistorted_points_;
    std::shared_ptr<Image_Data> image_data_;
    double timestamp_ = 0.0;
};