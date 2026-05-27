#pragma once
#include <iostream>
#include <unordered_map>
#include <vector>
#include <list>
#include <functional>

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

#include "yaml-cpp/yaml.h"

#include "./rc_vox.hpp"

/**
 * @brief 点云体素地图封装。
 *
 * 该模板类将体素容器、检索策略和地图参数组织在一起，
 * 对外提供插入、搜索和配置加载接口，供里程计主流程调用。
 */
template <template <typename, typename> class PointVoxelMapType, typename data_type, typename MapInfo> class RC_Vox_Map {
public:
    RC_Vox_Map() = default;

    RC_Vox_Map(const int &param_nearby_type, const double &range_x, const double &range_y, const double &range_z, const double &point_resolution = 0.5, const double &voxel_resolution = 0.5,
               const double &grid_resolution = 4.0, const int &point_pool_increm_size = 1000, const int &voxel_pool_increm_size = 1000, const int &grid_pool_increm_size = 1000)
        : param_nearby_type_(param_nearby_type)
        , point_resolution_(point_resolution)
        , voxel_resolution_(voxel_resolution)
        , grid_resolution_(grid_resolution)
        , point_voxel_map_(range_x, range_y, range_z, point_resolution, voxel_resolution, grid_resolution, point_pool_increm_size, voxel_pool_increm_size, grid_pool_increm_size)
    {
        generate_nearby_nodes();
    }

    void load_config_parameter(const std::string &system_dir)
    {
        std::string config_file = system_dir + "pointcloud_map/rc_vox_map.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file);

        int param_nearby_type;
        double param_range_x, param_range_y, param_range_z;
        double grid_resolution, voxel_resolution, point_resolution;
        int param_grid_pool_increm_size, param_voxel_pool_increm_size, param_point_pool_increm_size;
        int param_reserve_grid_size, param_reserve_voxel_size, param_reserve_point_size;
        try {
            param_nearby_type = config_node["param_nearby_type"].as<int>();

            param_range_x = config_node["param_range_x"].as<double>();
            param_range_y = config_node["param_range_y"].as<double>();
            param_range_z = config_node["param_range_z"].as<double>();

            grid_resolution = config_node["grid_resolution"].as<double>();
            voxel_resolution = config_node["voxel_resolution"].as<double>();
            point_resolution = config_node["point_resolution"].as<double>();

            param_grid_pool_increm_size = config_node["param_grid_pool_increm_size"].as<int>();
            param_voxel_pool_increm_size = config_node["param_voxel_pool_increm_size"].as<int>();
            param_point_pool_increm_size = config_node["param_point_pool_increm_size"].as<int>();

            param_reserve_grid_size = config_node["param_reserve_grid_size"].as<int>();
            param_reserve_voxel_size = config_node["param_reserve_voxel_size"].as<int>();
            param_reserve_point_size = config_node["param_reserve_point_size"].as<int>();
        } catch (...) {
            LOG(FATAL) << "[RC_Vox_Map]: file " << config_file << " has a bad conversion";
        }

        LOG(INFO) << std::endl
                  << "| --------------- [RC_Vox_Map parameter] --------------- |" << std::endl
                  << "[param_nearby_type] " << param_nearby_type << std::endl
                  << "[param_range_x] " << param_range_x << std::endl
                  << "[param_range_y] " << param_range_y << std::endl
                  << "[param_range_z] " << param_range_z << std::endl
                  << "[grid_resolution] " << grid_resolution << std::endl
                  << "[voxel_resolution] " << voxel_resolution << std::endl
                  << "[point_resolution] " << point_resolution << std::endl
                  << "[param_grid_pool_increm_size] " << param_grid_pool_increm_size << std::endl
                  << "[param_voxel_pool_increm_size] " << param_voxel_pool_increm_size << std::endl
                  << "[param_point_pool_increm_size] " << param_point_pool_increm_size << std::endl
                  << "[param_reserve_grid_size] " << param_reserve_grid_size << std::endl
                  << "[param_reserve_voxel_size] " << param_reserve_voxel_size << std::endl
                  << "[param_reserve_point_size] " << param_reserve_point_size << std::endl
                  << std::endl;

        set_parameter(param_nearby_type, param_range_x, param_range_y, param_range_z, point_resolution, voxel_resolution, grid_resolution, param_point_pool_increm_size, param_voxel_pool_increm_size,
                      param_grid_pool_increm_size);
        reserve(param_reserve_point_size, param_reserve_voxel_size, param_reserve_grid_size);
    }

    void set_parameter(const int &param_nearby_type, const double &range_x, const double &range_y, const double &range_z, const double &point_resolution = 0.5, const double &voxel_resolution = 0.5,
                       const double &grid_resolution = 4.0, const int &point_pool_increm_size = 1000, const int &voxel_pool_increm_size = 1000, const int &grid_pool_increm_size = 1000)
    {
        param_nearby_type_ = param_nearby_type;
        point_resolution_ = point_resolution;
        voxel_resolution_ = voxel_resolution;
        grid_resolution_ = grid_resolution;
        point_voxel_map_.set_parameter(range_x, range_y, range_z, point_resolution, voxel_resolution, grid_resolution, point_pool_increm_size, voxel_pool_increm_size, grid_pool_increm_size);
        generate_nearby_nodes();
    }

    void reserve(size_t point_size, size_t voxel_size, size_t grid_size)
    {
        point_voxel_map_.reserve(point_size, voxel_size, grid_size);
    }

#if MULTI_THREAD
    template <typename PclPointType> void add_points(const std::vector<PclPointType, Eigen::aligned_allocator<PclPointType>> &pts)
    {
        std::for_each(std::execution::unseq, pts.begin(), pts.end(), [this](const auto &pt) {
            // if(point_voxel_map_.insert_return_result(pt.x, pt.y, pt.z)) {
            point_voxel_map_.insert_return_result(pt.x, pt.y, pt.z)
                // point_voxel_map_.insert_nearby(pt.x, pt.y, pt.z, pt.x, pt.y, pt.z);
                for (int idx = 0; idx < 18 /* 26 */; idx++)
            {
                offset_tmp_ = nearby_voxel_offset_array_ + 3 * idx;
                point_voxel_map_.insert_nearby(pt.x + offset_tmp_[0], pt.y + offset_tmp_[1], pt.z + offset_tmp_[2], pt.x, pt.y, pt.z);
            }
            for (int idx = 0; idx < 18 /* 26 */; idx++) {
                offset_tmp_ = nearby_node_offset_array_3_ + 3 * idx;
                point_voxel_map_.insert_nearby_pt(pt.x + offset_tmp_[0], pt.y + offset_tmp_[1], pt.z + offset_tmp_[2], pt.x, pt.y, pt.z);
            }
            // }
        });
    }
#else
    template <typename PclPointType> void add_points(const std::vector<PclPointType, Eigen::aligned_allocator<PclPointType>> &pts)
    {
        if (abs(voxel_resolution_ - point_resolution_) < 0.01) {
            for (auto &pt : pts) {
                if (point_voxel_map_.insert_point(pt.x, pt.y, pt.z)) {
                    for (int idx = 0; idx < param_nearby_type_; idx++) {
                        offset_tmp_ = nearby_voxel_offset_array_ + 3 * idx;
                        point_voxel_map_.insert_point_to_nearby_voxel(pt.x + offset_tmp_[0], pt.y + offset_tmp_[1], pt.z + offset_tmp_[2], pt.x, pt.y, pt.z);
                    }
                }
            }
        } else {
            for (auto &pt : pts) {
                if (point_voxel_map_.insert_point(pt.x, pt.y, pt.z)) {
                    for (int idx = 0; idx < param_nearby_type_; idx++) {
                        offset_tmp_ = nearby_voxel_offset_array_ + 3 * idx;
                        point_voxel_map_.insert_point_to_nearby_voxel(pt.x + offset_tmp_[0], pt.y + offset_tmp_[1], pt.z + offset_tmp_[2], pt.x, pt.y, pt.z);
                    }
                    for (int idx = 0; idx < param_nearby_type_; idx++) {
                        offset_tmp_ = nearby_point_offset_array_ + 3 * idx;
                        point_voxel_map_.insert_point_to_nearby_point_seed(pt.x + offset_tmp_[0], pt.y + offset_tmp_[1], pt.z + offset_tmp_[2], pt.x, pt.y, pt.z);
                    }
                }
            }
        }
    }

    template <typename PclPointType> void add_points_auto_downsample(const std::vector<PclPointType, Eigen::aligned_allocator<PclPointType>> &pts)
    {
        if (abs(voxel_resolution_ - point_resolution_) < 0.01) {
            for (auto &pt : pts) {
                if (point_voxel_map_.insert_point_auto_downsample(pt.x, pt.y, pt.z)) {
                    for (int idx = 0; idx < param_nearby_type_; idx++) {
                        offset_tmp_ = nearby_voxel_offset_array_ + 3 * idx;
                        point_voxel_map_.insert_point_to_nearby_voxel(pt.x + offset_tmp_[0], pt.y + offset_tmp_[1], pt.z + offset_tmp_[2], pt.x, pt.y, pt.z);
                    }
                }
            }
        } else {
            for (auto &pt : pts) {
                if (point_voxel_map_.insert_point_auto_downsample(pt.x, pt.y, pt.z)) {
                    for (int idx = 0; idx < param_nearby_type_; idx++) {
                        offset_tmp_ = nearby_voxel_offset_array_ + 3 * idx;
                        point_voxel_map_.insert_point_to_nearby_voxel(pt.x + offset_tmp_[0], pt.y + offset_tmp_[1], pt.z + offset_tmp_[2], pt.x, pt.y, pt.z);
                    }
                    for (int idx = 0; idx < param_nearby_type_; idx++) {
                        offset_tmp_ = nearby_point_offset_array_ + 3 * idx;
                        point_voxel_map_.insert_point_to_nearby_point_seed(pt.x + offset_tmp_[0], pt.y + offset_tmp_[1], pt.z + offset_tmp_[2], pt.x, pt.y, pt.z);
                    }
                }
            }
        }
    }
#endif

    template <typename PclPointType>
    void get_closest_point(PclPointType &pt, std::vector<PclPointType, Eigen::aligned_allocator<PclPointType>> &closest_pt, Point_Seed<Dist_Node> *&point_seed, int max_num = 5, double max_range = 5.0)
    {
        closest_pt.clear();

        MapInfo *voxel_info_ptr_tmp_ = point_voxel_map_.find(pt.x, pt.y, pt.z);
        if (voxel_info_ptr_tmp_) {
            int nearest_idx;
            voxel_info_ptr_tmp_->get_closest_point_seed(pt.x, pt.y, pt.z, point_seed, nearest_idx);
            if (nearest_idx < 0) {
                point_seed = nullptr;
                return;
            }

            PclPointType pt_tmp;
            if (point_seed->get_flg_has_patch() || point_seed->size() < 3) {
                auto &heap_tmp = point_seed->heap_;
                pt_tmp.x = heap_tmp[nearest_idx].x_;
                pt_tmp.y = heap_tmp[nearest_idx].y_;
                pt_tmp.z = heap_tmp[nearest_idx].z_;
                closest_pt.emplace_back(pt_tmp);
            } else {
                auto &heap_tmp = point_seed->heap_;
                pt_tmp.x = heap_tmp[nearest_idx].x_;
                pt_tmp.y = heap_tmp[nearest_idx].y_;
                pt_tmp.z = heap_tmp[nearest_idx].z_;
                closest_pt.emplace_back(pt_tmp);
                for (int i = 0; i < nearest_idx; i++) {
                    pt_tmp.x = heap_tmp[i].x_;
                    pt_tmp.y = heap_tmp[i].y_;
                    pt_tmp.z = heap_tmp[i].z_;
                    closest_pt.emplace_back(pt_tmp);
                }
                for (int i = nearest_idx + 1; i < point_seed->size(); i++) {
                    pt_tmp.x = heap_tmp[i].x_;
                    pt_tmp.y = heap_tmp[i].y_;
                    pt_tmp.z = heap_tmp[i].z_;
                    closest_pt.emplace_back(pt_tmp);
                }
            }
        } else {
            point_seed = nullptr;
        }
    }

    void incremental_cut_map(const double &x, const double &y, const double &z)
    {
        point_voxel_map_.incremental_cut_map(x, y, z);
    }

    void get_all_pts(std::vector<MapInfo *> &pts)
    {
        point_voxel_map_.get_all_pts(pts);
    }

    double get_grid_resolution()
    {
        return point_voxel_map_.get_grid_resolution();
    }

    double get_point_resolution()
    {
        return point_resolution_;
    }

    data_type get_range_x()
    {
        return point_voxel_map_.get_range_x();
    }
    data_type get_range_y()
    {
        return point_voxel_map_.get_range_y();
    }
    data_type get_range_z()
    {
        return point_voxel_map_.get_range_z();
    }

private:
    void generate_nearby_nodes()
    {
        nearby_voxel_offset_array_ = new double[26 * 3];
        nearby_voxel_offset_array_[0 * 3 + 0] = -voxel_resolution_;
        nearby_voxel_offset_array_[0 * 3 + 1] = 0;
        nearby_voxel_offset_array_[0 * 3 + 2] = 0;

        nearby_voxel_offset_array_[1 * 3 + 0] = voxel_resolution_;
        nearby_voxel_offset_array_[1 * 3 + 1] = 0;
        nearby_voxel_offset_array_[1 * 3 + 2] = 0;

        nearby_voxel_offset_array_[2 * 3 + 0] = 0;
        nearby_voxel_offset_array_[2 * 3 + 1] = -voxel_resolution_;
        nearby_voxel_offset_array_[2 * 3 + 2] = 0;

        nearby_voxel_offset_array_[3 * 3 + 0] = 0;
        nearby_voxel_offset_array_[3 * 3 + 1] = voxel_resolution_;
        nearby_voxel_offset_array_[3 * 3 + 2] = 0;

        nearby_voxel_offset_array_[4 * 3 + 0] = 0;
        nearby_voxel_offset_array_[4 * 3 + 1] = 0;
        nearby_voxel_offset_array_[4 * 3 + 2] = -voxel_resolution_;

        nearby_voxel_offset_array_[5 * 3 + 0] = 0;
        nearby_voxel_offset_array_[5 * 3 + 1] = 0;
        nearby_voxel_offset_array_[5 * 3 + 2] = voxel_resolution_;

        nearby_voxel_offset_array_[6 * 3 + 0] = voxel_resolution_;
        nearby_voxel_offset_array_[6 * 3 + 1] = -voxel_resolution_;
        nearby_voxel_offset_array_[6 * 3 + 2] = 0;

        nearby_voxel_offset_array_[7 * 3 + 0] = -voxel_resolution_;
        nearby_voxel_offset_array_[7 * 3 + 1] = voxel_resolution_;
        nearby_voxel_offset_array_[7 * 3 + 2] = 0;

        nearby_voxel_offset_array_[8 * 3 + 0] = voxel_resolution_;
        nearby_voxel_offset_array_[8 * 3 + 1] = voxel_resolution_;
        nearby_voxel_offset_array_[8 * 3 + 2] = 0;

        nearby_voxel_offset_array_[9 * 3 + 0] = -voxel_resolution_;
        nearby_voxel_offset_array_[9 * 3 + 1] = -voxel_resolution_;
        nearby_voxel_offset_array_[9 * 3 + 2] = 0;

        nearby_voxel_offset_array_[10 * 3 + 0] = 0;
        nearby_voxel_offset_array_[10 * 3 + 1] = voxel_resolution_;
        nearby_voxel_offset_array_[10 * 3 + 2] = voxel_resolution_;

        nearby_voxel_offset_array_[11 * 3 + 0] = 0;
        nearby_voxel_offset_array_[11 * 3 + 1] = voxel_resolution_;
        nearby_voxel_offset_array_[11 * 3 + 2] = -voxel_resolution_;

        nearby_voxel_offset_array_[12 * 3 + 0] = 0;
        nearby_voxel_offset_array_[12 * 3 + 1] = -voxel_resolution_;
        nearby_voxel_offset_array_[12 * 3 + 2] = voxel_resolution_;

        nearby_voxel_offset_array_[13 * 3 + 0] = 0;
        nearby_voxel_offset_array_[13 * 3 + 1] = -voxel_resolution_;
        nearby_voxel_offset_array_[13 * 3 + 2] = -voxel_resolution_;

        nearby_voxel_offset_array_[14 * 3 + 0] = voxel_resolution_;
        nearby_voxel_offset_array_[14 * 3 + 1] = 0;
        nearby_voxel_offset_array_[14 * 3 + 2] = voxel_resolution_;

        nearby_voxel_offset_array_[15 * 3 + 0] = -voxel_resolution_;
        nearby_voxel_offset_array_[15 * 3 + 1] = 0;
        nearby_voxel_offset_array_[15 * 3 + 2] = voxel_resolution_;

        nearby_voxel_offset_array_[16 * 3 + 0] = -voxel_resolution_;
        nearby_voxel_offset_array_[16 * 3 + 1] = 0;
        nearby_voxel_offset_array_[16 * 3 + 2] = -voxel_resolution_;

        nearby_voxel_offset_array_[17 * 3 + 0] = voxel_resolution_;
        nearby_voxel_offset_array_[17 * 3 + 1] = 0;
        nearby_voxel_offset_array_[17 * 3 + 2] = -voxel_resolution_;

        nearby_voxel_offset_array_[18 * 3 + 0] = voxel_resolution_;
        nearby_voxel_offset_array_[18 * 3 + 1] = voxel_resolution_;
        nearby_voxel_offset_array_[18 * 3 + 2] = voxel_resolution_;

        nearby_voxel_offset_array_[19 * 3 + 0] = -voxel_resolution_;
        nearby_voxel_offset_array_[19 * 3 + 1] = voxel_resolution_;
        nearby_voxel_offset_array_[19 * 3 + 2] = voxel_resolution_;

        nearby_voxel_offset_array_[20 * 3 + 0] = voxel_resolution_;
        nearby_voxel_offset_array_[20 * 3 + 1] = -voxel_resolution_;
        nearby_voxel_offset_array_[20 * 3 + 2] = voxel_resolution_;

        nearby_voxel_offset_array_[21 * 3 + 0] = voxel_resolution_;
        nearby_voxel_offset_array_[21 * 3 + 1] = voxel_resolution_;
        nearby_voxel_offset_array_[21 * 3 + 2] = -voxel_resolution_;

        nearby_voxel_offset_array_[22 * 3 + 0] = -voxel_resolution_;
        nearby_voxel_offset_array_[22 * 3 + 1] = -voxel_resolution_;
        nearby_voxel_offset_array_[22 * 3 + 2] = voxel_resolution_;

        nearby_voxel_offset_array_[23 * 3 + 0] = -voxel_resolution_;
        nearby_voxel_offset_array_[23 * 3 + 1] = voxel_resolution_;
        nearby_voxel_offset_array_[23 * 3 + 2] = -voxel_resolution_;

        nearby_voxel_offset_array_[24 * 3 + 0] = voxel_resolution_;
        nearby_voxel_offset_array_[24 * 3 + 1] = -voxel_resolution_;
        nearby_voxel_offset_array_[24 * 3 + 2] = -voxel_resolution_;

        nearby_voxel_offset_array_[25 * 3 + 0] = -voxel_resolution_;
        nearby_voxel_offset_array_[25 * 3 + 1] = -voxel_resolution_;
        nearby_voxel_offset_array_[25 * 3 + 2] = -voxel_resolution_;

        nearby_point_offset_array_ = new double[26 * 3];
        nearby_point_offset_array_[0 * 3 + 0] = -point_resolution_;
        nearby_point_offset_array_[0 * 3 + 1] = 0;
        nearby_point_offset_array_[0 * 3 + 2] = 0;

        nearby_point_offset_array_[1 * 3 + 0] = point_resolution_;
        nearby_point_offset_array_[1 * 3 + 1] = 0;
        nearby_point_offset_array_[1 * 3 + 2] = 0;

        nearby_point_offset_array_[2 * 3 + 0] = 0;
        nearby_point_offset_array_[2 * 3 + 1] = -point_resolution_;
        nearby_point_offset_array_[2 * 3 + 2] = 0;

        nearby_point_offset_array_[3 * 3 + 0] = 0;
        nearby_point_offset_array_[3 * 3 + 1] = point_resolution_;
        nearby_point_offset_array_[3 * 3 + 2] = 0;

        nearby_point_offset_array_[4 * 3 + 0] = 0;
        nearby_point_offset_array_[4 * 3 + 1] = 0;
        nearby_point_offset_array_[4 * 3 + 2] = -point_resolution_;

        nearby_point_offset_array_[5 * 3 + 0] = 0;
        nearby_point_offset_array_[5 * 3 + 1] = 0;
        nearby_point_offset_array_[5 * 3 + 2] = point_resolution_;

        nearby_point_offset_array_[6 * 3 + 0] = point_resolution_;
        nearby_point_offset_array_[6 * 3 + 1] = -point_resolution_;
        nearby_point_offset_array_[6 * 3 + 2] = 0;

        nearby_point_offset_array_[7 * 3 + 0] = -point_resolution_;
        nearby_point_offset_array_[7 * 3 + 1] = point_resolution_;
        nearby_point_offset_array_[7 * 3 + 2] = 0;

        nearby_point_offset_array_[8 * 3 + 0] = point_resolution_;
        nearby_point_offset_array_[8 * 3 + 1] = point_resolution_;
        nearby_point_offset_array_[8 * 3 + 2] = 0;

        nearby_point_offset_array_[9 * 3 + 0] = -point_resolution_;
        nearby_point_offset_array_[9 * 3 + 1] = -point_resolution_;
        nearby_point_offset_array_[9 * 3 + 2] = 0;

        nearby_point_offset_array_[10 * 3 + 0] = 0;
        nearby_point_offset_array_[10 * 3 + 1] = point_resolution_;
        nearby_point_offset_array_[10 * 3 + 2] = point_resolution_;

        nearby_point_offset_array_[11 * 3 + 0] = 0;
        nearby_point_offset_array_[11 * 3 + 1] = point_resolution_;
        nearby_point_offset_array_[11 * 3 + 2] = -point_resolution_;

        nearby_point_offset_array_[12 * 3 + 0] = 0;
        nearby_point_offset_array_[12 * 3 + 1] = -point_resolution_;
        nearby_point_offset_array_[12 * 3 + 2] = point_resolution_;

        nearby_point_offset_array_[13 * 3 + 0] = 0;
        nearby_point_offset_array_[13 * 3 + 1] = -point_resolution_;
        nearby_point_offset_array_[13 * 3 + 2] = -point_resolution_;

        nearby_point_offset_array_[14 * 3 + 0] = point_resolution_;
        nearby_point_offset_array_[14 * 3 + 1] = 0;
        nearby_point_offset_array_[14 * 3 + 2] = point_resolution_;

        nearby_point_offset_array_[15 * 3 + 0] = -point_resolution_;
        nearby_point_offset_array_[15 * 3 + 1] = 0;
        nearby_point_offset_array_[15 * 3 + 2] = point_resolution_;

        nearby_point_offset_array_[16 * 3 + 0] = -point_resolution_;
        nearby_point_offset_array_[16 * 3 + 1] = 0;
        nearby_point_offset_array_[16 * 3 + 2] = -point_resolution_;

        nearby_point_offset_array_[17 * 3 + 0] = point_resolution_;
        nearby_point_offset_array_[17 * 3 + 1] = 0;
        nearby_point_offset_array_[17 * 3 + 2] = -point_resolution_;

        nearby_point_offset_array_[18 * 3 + 0] = point_resolution_;
        nearby_point_offset_array_[18 * 3 + 1] = point_resolution_;
        nearby_point_offset_array_[18 * 3 + 2] = point_resolution_;

        nearby_point_offset_array_[19 * 3 + 0] = -point_resolution_;
        nearby_point_offset_array_[19 * 3 + 1] = point_resolution_;
        nearby_point_offset_array_[19 * 3 + 2] = point_resolution_;

        nearby_point_offset_array_[20 * 3 + 0] = point_resolution_;
        nearby_point_offset_array_[20 * 3 + 1] = -point_resolution_;
        nearby_point_offset_array_[20 * 3 + 2] = point_resolution_;

        nearby_point_offset_array_[21 * 3 + 0] = point_resolution_;
        nearby_point_offset_array_[21 * 3 + 1] = point_resolution_;
        nearby_point_offset_array_[21 * 3 + 2] = -point_resolution_;

        nearby_point_offset_array_[22 * 3 + 0] = -point_resolution_;
        nearby_point_offset_array_[22 * 3 + 1] = -point_resolution_;
        nearby_point_offset_array_[22 * 3 + 2] = point_resolution_;

        nearby_point_offset_array_[23 * 3 + 0] = -point_resolution_;
        nearby_point_offset_array_[23 * 3 + 1] = point_resolution_;
        nearby_point_offset_array_[23 * 3 + 2] = -point_resolution_;

        nearby_point_offset_array_[24 * 3 + 0] = point_resolution_;
        nearby_point_offset_array_[24 * 3 + 1] = -point_resolution_;
        nearby_point_offset_array_[24 * 3 + 2] = -point_resolution_;

        nearby_point_offset_array_[25 * 3 + 0] = -point_resolution_;
        nearby_point_offset_array_[25 * 3 + 1] = -point_resolution_;
        nearby_point_offset_array_[25 * 3 + 2] = -point_resolution_;
    }

private:
    double *nearby_voxel_offset_array_;
    double *nearby_point_offset_array_;

    int param_nearby_type_;
    double grid_resolution_, voxel_resolution_, point_resolution_;

    // TODO:
    PointVoxelMapType<data_type, MapInfo> point_voxel_map_;
    double *offset_tmp_;
};