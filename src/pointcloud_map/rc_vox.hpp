#pragma once
#include <iostream>
#include <unordered_map>
#include <vector>
#include <list>

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

#include "../memory_pool/custum_memory_pool.hpp"

/**
 * @brief 体素单元与近邻检索基础实现。
 *
 * 该文件提供RC-Vox地图的底层数据结构和距离搜索相关操作，
 * 被上层 `RC_Vox_Map` 封装调用。
 */

#define SAFE_ALLOCATE
#define SAFE_INSERT_AND_FIND
// #define Calculation_Simplify
#define FIXED_RESOLUTION

struct Dist_Node {
    double x_, y_, z_;
    float dist_;
    bool operator<(const Dist_Node &a) const
    {
        dist_ < a.dist_;
    }
};

template <typename Info_Type> class Point_Seed {
public:
    Point_Seed(int max_capacity = 5)
        : cap(max_capacity)
        , heap_size(0)
        , flg_build_(false)
        , flg_has_patch_(false)
        , flg_has_point_(false)
    {
        heap_ = new Info_Type[max_capacity];
    }

    ~Point_Seed()
    {
        delete[] heap_;
    }

    bool get_flg_has_patch()
    {
        return flg_has_patch_;
    }

    void set_flg_has_patch()
    {
        flg_has_patch_ = true;
    }

    void set_patch(const Eigen::Vector4f &patch)
    {
        patch_ = patch;
    }

    Eigen::Vector4f &get_patch()
    {
        return patch_;
    }

    int size()
    {
        return heap_size;
    }

    void clear()
    {
        heap_size = 0;
        flg_build_ = false;
        flg_has_point_ = false;
    }

    void reset()
    {
        heap_size = 0;
        flg_build_ = false;
        flg_has_patch_ = false;
        flg_has_point_ = false;
    }

    void set_position(const double &x, const double &y, const double &z)
    {
        position_x_ = x;
        position_y_ = y;
        position_z_ = z;
    }

    Info_Type *top()
    {
        return heap_[0];
    }

    void pop()
    {
        // if (heap_size == 0) return;
        heap_[0] = heap_[heap_size - 1];
        heap_size--;
        move_down(0);
        return;
    }

    void push(const double &x, const double &y, const double &z)
    {
#ifdef Calculation_Simplify
        float dist = abs(x - position_x_) + abs(y - position_y_) + abs(z - position_z_);
#else
        float dist = (x - position_x_) * (x - position_x_) + (y - position_y_) * (y - position_y_) + (z - position_z_) * (z - position_z_);
#endif

        if (heap_size < cap) {
            heap_[heap_size].x_ = x;
            heap_[heap_size].y_ = y;
            heap_[heap_size].z_ = z;
            heap_[heap_size].dist_ = dist;
            heap_size++;
            flg_has_patch_ = false;
        } else {
            if (!flg_build_) {
                flg_build_ = true;
                build();
            }
            if ((heap_[0].dist_ - dist) > 0.1) {
                pop();
            } else {
                return;
            }
            heap_[heap_size].x_ = x;
            heap_[heap_size].y_ = y;
            heap_[heap_size].z_ = z;
            heap_[heap_size].dist_ = dist;
            float_up(heap_size);
            heap_size++;
            flg_has_patch_ = false;
        }
    }

    bool push_auto_downsample(const double &x, const double &y, const double &z)
    {
        if (flg_has_point_) {
            return false;
        }
        flg_has_point_ = true;

#ifdef Calculation_Simplify
        float dist = abs(x - position_x_) + abs(y - position_y_) + abs(z - position_z_);
#else
        float dist = (x - position_x_) * (x - position_x_) + (y - position_y_) * (y - position_y_) + (z - position_z_) * (z - position_z_);
#endif

        if (heap_size < cap) {
            heap_[heap_size].x_ = x;
            heap_[heap_size].y_ = y;
            heap_[heap_size].z_ = z;
            heap_[heap_size].dist_ = dist;
            heap_size++;
            flg_has_patch_ = false;
        } else {
            if (!flg_build_) {
                flg_build_ = true;
                build();
            }
            if ((heap_[0].dist_ - dist) > 0.1) {
                pop();
            } else {
                return true;
            }
            heap_[heap_size].x_ = x;
            heap_[heap_size].y_ = y;
            heap_[heap_size].z_ = z;
            heap_[heap_size].dist_ = dist;
            float_up(heap_size);
            heap_size++;
            flg_has_patch_ = false;
        }
        return true;
    }

    void get_closest_point_idx(const double &x, const double &y, const double &z, int &idx)
    {
        if (heap_size > 0) {
#ifdef Calculation_Simplify
            float dist_tmp_, dist_min_ = abs(heap_[0].x_ - x) + abs(heap_[0].y_ - y) + abs(heap_[0].z_ - z);
#else
            float dist_tmp_, dist_min_ = (heap_[0].x_ - x) * (heap_[0].x_ - x) + (heap_[0].y_ - y) * (heap_[0].y_ - y) + (heap_[0].z_ - z) * (heap_[0].z_ - z);
#endif
            idx = 0;
            for (int i = 1; i < heap_size; i++) {
#ifdef Calculation_Simplify
                dist_tmp_ = abs(heap_[i].x_ - x) + abs(heap_[i].y_ - y) + abs(heap_[i].z_ - z);
#else
                dist_tmp_ = (heap_[i].x_ - x) * (heap_[i].x_ - x) + (heap_[i].y_ - y) * (heap_[i].y_ - y) + (heap_[i].z_ - z) * (heap_[i].z_ - z);
#endif
                if (dist_tmp_ < dist_min_) {
                    dist_min_ = dist_tmp_;
                    idx = i;
                }
            }
        } else {
            idx = -1;
        }
    }

private:
    void build()
    {
        for (int i = (heap_size - 2) / 2; i >= 0; i--) {
            move_down(i);
        }
    }

    // TODO: 改为指针的交换操作
    void move_down(int heap_index)
    {
        int l = heap_index * 2 + 1;
        Info_Type &tmp = heap_[heap_index];
        while (l < heap_size) {
            if (l + 1 < heap_size && heap_[l] < heap_[l + 1])
                l++;
            if (tmp < heap_[l]) {
                heap_[heap_index] = heap_[l];
                heap_index = l;
                l = heap_index * 2 + 1;
            } else
                break;
        }
        heap_[heap_index] = tmp;
        return;
    }

    void float_up(int heap_index)
    {
        int ancestor = (heap_index - 1) / 2;
        Info_Type &tmp = heap_[heap_index];
        while (heap_index > 0) {
            if (heap_[ancestor] < tmp) {
                heap_[heap_index] = heap_[ancestor];
                heap_index = ancestor;
                ancestor = (heap_index - 1) / 2;
            } else
                break;
        }
        heap_[heap_index] = tmp;
        return;
    }

public:
    double position_x_, position_y_, position_z_;
    Info_Type *heap_;

private:
    int heap_size = 0;
    int cap = 0; // 5
    bool flg_build_;
    bool flg_has_patch_;
    bool flg_has_point_;
    Eigen::Vector4f patch_;
};

template <typename KeyType> class Voxel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Voxel()
        : v_point_seed_ptr_(9)
    {
    }

#ifdef FIXED_RESOLUTION
    static void set_parameter(const double &voxel_resolution, const double &point_resolution)
    {
        voxel_resolution_ = voxel_resolution;
        voxel_resolution_d2_ = voxel_resolution_ / 2.0;
        point_resolution_ = point_resolution;
        point_resolution_d2_ = point_resolution_ / 2.0;
        point_num_per_dim_ = voxel_resolution_ / point_resolution_;
        point_num_per_dim_sqr_ = point_num_per_dim_ * point_num_per_dim_;
    }
#else
    static void set_parameter(const double &voxel_resolution)
    {
        voxel_resolution_ = voxel_resolution;
        voxel_resolution_d2_ = voxel_resolution_ / 2.0;
    }
#endif

    void reset()
    {
        flg_has_point_ = false;
        if (point_num_per_dim_ == 1) {
            point_seed_.reset();
        } else {
            for (auto &item : v_point_seed_ptr_) {
                if (!item)
                    break;
                point_seed_pool_.deallocate(item);
            }
            point_seed_.reset();
        }
    }

#ifdef FIXED_RESOLUTION
    void reset(const double &x, const double &y, const double &z)
    {
        if (point_num_per_dim_ == 1) {
            point_seed_.set_position(x + voxel_resolution_d2_, y + voxel_resolution_d2_, z + voxel_resolution_d2_);
        } else {
            point_seed_.set_position(x + voxel_resolution_d2_, y + voxel_resolution_d2_, z + voxel_resolution_d2_);
            min_corner_pos_x_ = x;
            min_corner_pos_y_ = y;
            min_corner_pos_z_ = z;

            int idx;
            double x_0 = x + point_resolution_d2_, y_0 = y + point_resolution_d2_, z_0 = z + point_resolution_d2_;
            for (int i = 0; i < point_num_per_dim_; i++) {
                for (int j = 0; j < point_num_per_dim_; j++) {
                    for (int k = 0; k < point_num_per_dim_; k++) {
                        idx = k * point_num_per_dim_sqr_ + j * point_num_per_dim_ + i;
#ifdef SAFE_ALLOCATE
                        v_point_seed_ptr_[idx] = point_seed_pool_.allocate_safe();
#else
                        v_point_seed_ptr_[idx] = point_seed_pool_.allocate_fast();
#endif
                        v_point_seed_ptr_[idx]->set_position(x_0 + point_resolution_ * i, y_0 + point_resolution_ * j, z_0 + point_resolution_ * k);
                    }
                }
            }
        }
    }
#else
    void reset(const double &x, const double &y, const double &z, const double &point_resolution)
    {
        point_resolution_ = point_resolution;
        point_resolution_d2_ = point_resolution_ / 2.0;
        point_num_per_dim_ = voxel_resolution_ / point_resolution_;
        point_num_per_dim_sqr_ = point_num_per_dim_ * point_num_per_dim_;
        if (point_num_per_dim_ == 1) {
            point_seed_.set_position(x + voxel_resolution_d2_, y + voxel_resolution_d2_, z + voxel_resolution_d2_);
        } else {
            point_seed_.set_position(x + voxel_resolution_d2_, y + voxel_resolution_d2_, z + voxel_resolution_d2_);
            min_corner_pos_x_ = x;
            min_corner_pos_y_ = y;
            min_corner_pos_z_ = z;

            int idx;
            double x_0 = x + point_resolution_d2_, y_0 = y + point_resolution_d2_, z_0 = z + point_resolution_d2_;
            for (int i = 0; i < point_num_per_dim_; i++) {
                for (int j = 0; j < point_num_per_dim_; j++) {
                    for (int k = 0; k < point_num_per_dim_; k++) {
                        idx = k * point_num_per_dim_sqr_ + j * point_num_per_dim_ + i;
#ifdef SAFE_ALLOCATE
                        v_point_seed_ptr_[idx] = point_seed_pool_.allocate_safe();
#else
                        v_point_seed_ptr_[idx] = point_seed_pool_.allocate_fast();
#endif
                        v_point_seed_ptr_[idx]->set_position(x_0 + point_resolution_ * i, y_0 + point_resolution_ * j, z_0 + point_resolution_ * k);
                    }
                }
            }
        }
    }
#endif

    void get_closest_point_seed(const double &x, const double &y, const double &z, Point_Seed<Dist_Node> *&point_seed_output, int &closest_point_idx_output)
    {
        if (point_num_per_dim_ == 1 || !flg_has_point_) {
            point_seed_output = &point_seed_;
            point_seed_.get_closest_point_idx(x, y, z, closest_point_idx_output);
        } else {
            double dist_min = 10000, dist_tmp;
            int id_min = -1, id = 0;
            for (auto &item : v_point_seed_ptr_) {
                if (item == nullptr)
                    break;
                if (item->size() > 0) {
#ifdef Calculation_Simplify
                    dist_tmp = abs(x - item.position_x_) + abs(y - item.position_y_) + abs(z - item.position_z_);
#else
                    dist_tmp = (x - item->position_x_) * (x - item->position_x_) + (y - item->position_y_) * (y - item->position_y_) + (z - item->position_z_) * (z - item->position_z_);
#endif

                    if (dist_tmp < dist_min) {
                        dist_min = dist_tmp;
                        id_min = id;
                    }
                }
                id++;
            }
            if (id_min < 0) {
                point_seed_output = &point_seed_;
                point_seed_.get_closest_point_idx(x, y, z, closest_point_idx_output);
            } else {
                point_seed_output = v_point_seed_ptr_[id_min];
                point_seed_output->get_closest_point_idx(x, y, z, closest_point_idx_output);
            }
        }
    }

    void add_point(const double &x, const double &y, const double &z)
    {
        if (point_num_per_dim_ == 1) {
            point_seed_.push(x, y, z);
        } else {
            flg_has_point_ = true;
            int idx = int((z - min_corner_pos_z_) / point_resolution_) * point_num_per_dim_sqr_ + int((y - min_corner_pos_y_) / point_resolution_) * point_num_per_dim_ +
                      int((x - min_corner_pos_x_) / point_resolution_);
            v_point_seed_ptr_[idx]->push(x, y, z);
            point_seed_.push(x, y, z);
        }
    }

    bool add_point_auto_downsample(const double &x, const double &y, const double &z)
    {
        if (point_num_per_dim_ == 1) {
            if (flg_has_point_) {
                return false;
            } else {
                flg_has_point_ = true;
                point_seed_.push(x, y, z);
                return true;
            }
        } else {
            flg_has_point_ = true;
            int idx = int((z - min_corner_pos_z_) / point_resolution_) * point_num_per_dim_sqr_ + int((y - min_corner_pos_y_) / point_resolution_) * point_num_per_dim_ +
                      int((x - min_corner_pos_x_) / point_resolution_);
            if (v_point_seed_ptr_[idx]->push_auto_downsample(x, y, z)) {
                point_seed_.push(x, y, z);
                return true;
            } else {
                return false;
            }
        }
    }

    void add_point_nearby_voxel(const double &x, const double &y, const double &z)
    {
        point_seed_.push(x, y, z);
    }

    void add_point_nearby_pt(const double &x, const double &y, const double &z, const double &x_info, const double &y_info, const double &z_info)
    {
        int idx = int((z - min_corner_pos_z_) / point_resolution_) * point_num_per_dim_sqr_ + int((y - min_corner_pos_y_) / point_resolution_) * point_num_per_dim_ +
                  int((x - min_corner_pos_x_) / point_resolution_);
        v_point_seed_ptr_[idx]->push(x_info, y_info, z_info);
    }

public:
    bool flg_has_point_ = false;
    double min_corner_pos_x_, min_corner_pos_y_, min_corner_pos_z_;
    Point_Seed<Dist_Node> point_seed_;
    std::vector<Point_Seed<Dist_Node> *> v_point_seed_ptr_;

    static Custom_Memory_Pool<Point_Seed<Dist_Node>> point_seed_pool_;
    static double voxel_resolution_;
    static double voxel_resolution_d2_;

#ifdef FIXED_RESOLUTION
    static double point_resolution_;
    static double point_resolution_d2_;
    static int point_num_per_dim_;
    static int point_num_per_dim_sqr_;
#else
    double point_resolution_;
    double point_resolution_d2_;
    int point_num_per_dim_;
    int point_num_per_dim_sqr_;
#endif
};

template <typename KeyType> Custom_Memory_Pool<Point_Seed<Dist_Node>> Voxel<KeyType>::point_seed_pool_;

template <typename KeyType> double Voxel<KeyType>::voxel_resolution_;

template <typename KeyType> double Voxel<KeyType>::voxel_resolution_d2_;

#ifdef FIXED_RESOLUTION
template <typename KeyType> int Voxel<KeyType>::point_num_per_dim_;

template <typename KeyType> int Voxel<KeyType>::point_num_per_dim_sqr_;

template <typename KeyType> double Voxel<KeyType>::point_resolution_;

template <typename KeyType> double Voxel<KeyType>::point_resolution_d2_;
#endif

template <typename data_type = int, typename MapInfo = void> class RC_Vox_Bottom_Layer {
public:
    RC_Vox_Bottom_Layer()
    {
        map_content_.resize(content_num_per_dim_sqr_ * content_num_per_dim_, nullptr);
    }

    static void set_parameter(const double &grid_resolution, const double &voxel_resolution)
    {
        grid_resolution_ = grid_resolution;
        voxel_resolution_ = voxel_resolution;
        content_num_per_dim_ = grid_resolution_ / voxel_resolution_; // TODO: 最好可以整除，不能的话如何处理。
        content_num_per_dim_sqr_ = content_num_per_dim_ * content_num_per_dim_;
    }

    void set_min_corner_pos(const double &min_corner_pos_x, const double &min_corner_pos_y, const double &min_corner_pos_z)
    {
        min_corner_pos_x_ = min_corner_pos_x;
        min_corner_pos_y_ = min_corner_pos_y;
        min_corner_pos_z_ = min_corner_pos_z;
    }

    void reset()
    {
        for (auto &ptr : map_content_) {
            if (ptr) {
                voxel_pool_.deallocate(ptr);
                ptr = nullptr;
            }
        }
    }

    MapInfo *find(const double &x, const double &y, const double &z)
    {
        return map_content_[content_num_per_dim_sqr_ * data_type((z - min_corner_pos_z_) / voxel_resolution_) + content_num_per_dim_ * data_type((y - min_corner_pos_y_) / voxel_resolution_) +
                            data_type((x - min_corner_pos_x_) / voxel_resolution_)];
    }

    void insert_point(const double &x, const double &y, const double &z)
    {
        pos_x_tmp_ = (x - min_corner_pos_x_) / voxel_resolution_;
        pos_y_tmp_ = (y - min_corner_pos_y_) / voxel_resolution_;
        pos_z_tmp_ = (z - min_corner_pos_z_) / voxel_resolution_;
        key_tmp_ = content_num_per_dim_sqr_ * pos_z_tmp_ + content_num_per_dim_ * pos_y_tmp_ + pos_x_tmp_;

        if (!map_content_[key_tmp_]) {
#ifdef SAFE_ALLOCATE
            map_content_[key_tmp_] = voxel_pool_.allocate_safe();
#else
            map_content_[key_tmp_] = voxel_pool_.allocate_fast();
#endif
            map_content_[key_tmp_]->reset(pos_x_tmp_ * voxel_resolution_ + min_corner_pos_x_, pos_y_tmp_ * voxel_resolution_ + min_corner_pos_y_, pos_z_tmp_ * voxel_resolution_ + min_corner_pos_z_);
        }

        map_content_[key_tmp_]->add_point(x, y, z);
    }

    bool insert_point_auto_downsample(const double &x, const double &y, const double &z)
    {
        pos_x_tmp_ = (x - min_corner_pos_x_) / voxel_resolution_;
        pos_y_tmp_ = (y - min_corner_pos_y_) / voxel_resolution_;
        pos_z_tmp_ = (z - min_corner_pos_z_) / voxel_resolution_;
        key_tmp_ = content_num_per_dim_sqr_ * pos_z_tmp_ + content_num_per_dim_ * pos_y_tmp_ + pos_x_tmp_;

        if (!map_content_[key_tmp_]) {
#ifdef SAFE_ALLOCATE
            map_content_[key_tmp_] = voxel_pool_.allocate_safe();
#else
            map_content_[key_tmp_] = voxel_pool_.allocate_fast();
#endif
            map_content_[key_tmp_]->reset(pos_x_tmp_ * voxel_resolution_ + min_corner_pos_x_, pos_y_tmp_ * voxel_resolution_ + min_corner_pos_y_, pos_z_tmp_ * voxel_resolution_ + min_corner_pos_z_);
        }

        return map_content_[key_tmp_]->add_point_auto_downsample(x, y, z);
    }

    void insert_point_to_nearby_voxel(const double &x, const double &y, const double &z, const double &x_info, const double &y_info, const double &z_info)
    {
        pos_x_tmp_ = (x - min_corner_pos_x_) / voxel_resolution_;
        pos_y_tmp_ = (y - min_corner_pos_y_) / voxel_resolution_;
        pos_z_tmp_ = (z - min_corner_pos_z_) / voxel_resolution_;
        key_tmp_ = content_num_per_dim_sqr_ * pos_z_tmp_ + content_num_per_dim_ * pos_y_tmp_ + pos_x_tmp_;

        if (!map_content_[key_tmp_]) {
#ifdef SAFE_ALLOCATE
            map_content_[key_tmp_] = voxel_pool_.allocate_safe();
#else
            map_content_[key_tmp_] = voxel_pool_.allocate_fast();
#endif
            map_content_[key_tmp_]->reset(pos_x_tmp_ * voxel_resolution_ + min_corner_pos_x_, pos_y_tmp_ * voxel_resolution_ + min_corner_pos_y_, pos_z_tmp_ * voxel_resolution_ + min_corner_pos_z_);
        }

        map_content_[key_tmp_]->add_point_nearby_voxel(x_info, y_info, z_info);
    }

    void insert_point_to_nearby_point_seed(const double &x, const double &y, const double &z, const double &x_info, const double &y_info, const double &z_info)
    {
        pos_x_tmp_ = (x - min_corner_pos_x_) / voxel_resolution_;
        pos_y_tmp_ = (y - min_corner_pos_y_) / voxel_resolution_;
        pos_z_tmp_ = (z - min_corner_pos_z_) / voxel_resolution_;
        key_tmp_ = content_num_per_dim_sqr_ * pos_z_tmp_ + content_num_per_dim_ * pos_y_tmp_ + pos_x_tmp_;

        if (!map_content_[key_tmp_]) {
#ifdef SAFE_ALLOCATE
            map_content_[key_tmp_] = voxel_pool_.allocate_safe();
#else
            map_content_[key_tmp_] = voxel_pool_.allocate_fast();
#endif
            map_content_[key_tmp_]->reset(pos_x_tmp_ * voxel_resolution_ + min_corner_pos_x_, pos_y_tmp_ * voxel_resolution_ + min_corner_pos_y_, pos_z_tmp_ * voxel_resolution_ + min_corner_pos_z_);
        }

        map_content_[key_tmp_]->add_point_nearby_pt(x, y, z, x_info, y_info, z_info);
    }

    void get_all_pts(std::vector<MapInfo *> &pts)
    {
        for (auto &ptr : map_content_) {
            if (ptr) {
                pts.push_back(ptr);
            }
        }
    }

    double get_grid_resolution()
    {
        return grid_resolution_;
    }

    Eigen::Vector3d get_min_corner_pos()
    {
        return Eigen::Vector3d(min_corner_pos_x_, min_corner_pos_y_, min_corner_pos_z_);
    }

public:
    static Custom_Memory_Pool<MapInfo> voxel_pool_;
    static double grid_resolution_;
    static double voxel_resolution_;
    static data_type content_num_per_dim_;
    static data_type content_num_per_dim_sqr_;

private:
    std::vector<MapInfo *> map_content_;

    double min_corner_pos_x_;
    double min_corner_pos_y_;
    double min_corner_pos_z_;

    data_type key_tmp_;
    data_type pos_x_tmp_, pos_y_tmp_, pos_z_tmp_;
};

template <typename data_type, typename MapInfo> Custom_Memory_Pool<MapInfo> RC_Vox_Bottom_Layer<data_type, MapInfo>::voxel_pool_;

template <typename data_type, typename MapInfo> double RC_Vox_Bottom_Layer<data_type, MapInfo>::grid_resolution_;

template <typename data_type, typename MapInfo> double RC_Vox_Bottom_Layer<data_type, MapInfo>::voxel_resolution_;

template <typename data_type, typename MapInfo> data_type RC_Vox_Bottom_Layer<data_type, MapInfo>::content_num_per_dim_;

template <typename data_type, typename MapInfo> data_type RC_Vox_Bottom_Layer<data_type, MapInfo>::content_num_per_dim_sqr_;

template <typename data_type = int, typename MapInfo = void> class RC_Vox {
public:
    RC_Vox() = default;

    RC_Vox(const double &range_x, const double &range_y, const double &range_z, const double &point_resolution = 0.5, const double &voxel_resolution = 0.5, const double &grid_resolution = 4.0,
           const int &point_pool_increm_size = 1000, const int &voxel_pool_increm_size = 1000, const int &grid_pool_increm_size = 1000)
        : content_num_dim_x_(range_x / grid_resolution)
        , content_num_dim_y_(range_y / grid_resolution)
        , content_num_dim_z_(range_z / grid_resolution)
        , point_resolution_(point_resolution)
        , voxel_resolution_(voxel_resolution)
        , grid_resolution_(grid_resolution)
        , flg_need_init_robot_position_(true)
    {
        content_num_dim_xy_ = content_num_dim_x_ * content_num_dim_y_;
        map_content_.resize(content_num_dim_xy_ * content_num_dim_z_, nullptr);

#ifdef FIXED_RESOLUTION
        MapInfo::set_parameter(voxel_resolution_, point_resolution_);
#else
        MapInfo::set_parameter(voxel_resolution_);
#endif

        RC_Vox_Bottom_Layer<data_type, MapInfo>::set_parameter(grid_resolution_, voxel_resolution_);

        MapInfo::point_seed_pool_.set_increm_size(point_pool_increm_size);
        MapInfo::point_seed_pool_.set_construct_func([]() { return new Point_Seed<Dist_Node>(); });

        RC_Vox_Bottom_Layer<data_type, MapInfo>::voxel_pool_.set_increm_size(voxel_pool_increm_size);
        RC_Vox_Bottom_Layer<data_type, MapInfo>::voxel_pool_.set_construct_func([]() { return new MapInfo(); });

        grid_pool_.set_increm_size(grid_pool_increm_size);
        grid_pool_.set_construct_func([]() { return new RC_Vox_Bottom_Layer<data_type, MapInfo>(); });
    }

    void set_parameter(const double &range_x, const double &range_y, const double &range_z, const double &point_resolution = 0.5, const double &voxel_resolution = 0.5,
                       const double &grid_resolution = 4.0, const int &point_pool_increm_size = 1000, const int &voxel_pool_increm_size = 1000, const int &grid_pool_increm_size = 1000)
    {
        content_num_dim_x_ = range_x / grid_resolution;
        content_num_dim_y_ = range_y / grid_resolution;
        content_num_dim_z_ = range_z / grid_resolution;
        point_resolution_ = point_resolution;
        voxel_resolution_ = voxel_resolution;
        grid_resolution_ = grid_resolution;
        flg_need_init_robot_position_ = true;

        content_num_dim_xy_ = content_num_dim_x_ * content_num_dim_y_;
        map_content_.clear();
        map_content_.resize(content_num_dim_xy_ * content_num_dim_z_, nullptr);

#ifdef FIXED_RESOLUTION
        MapInfo::set_parameter(voxel_resolution_, point_resolution_);
#else
        MapInfo::set_parameter(voxel_resolution_);
#endif

        RC_Vox_Bottom_Layer<data_type, MapInfo>::set_parameter(grid_resolution_, voxel_resolution_);

        MapInfo::point_seed_pool_.set_increm_size(point_pool_increm_size);
        MapInfo::point_seed_pool_.set_construct_func([]() { return new Point_Seed<Dist_Node>(); });

        RC_Vox_Bottom_Layer<data_type, MapInfo>::voxel_pool_.set_increm_size(voxel_pool_increm_size);
        RC_Vox_Bottom_Layer<data_type, MapInfo>::voxel_pool_.set_construct_func([]() { return new MapInfo(); });

        grid_pool_.set_increm_size(grid_pool_increm_size);
        grid_pool_.set_construct_func([]() { return new RC_Vox_Bottom_Layer<data_type, MapInfo>(); });
    }

    void reserve(size_t point_size, size_t voxel_size, size_t grid_size)
    {
        MapInfo::point_seed_pool_.expand_pool(point_size);
        grid_pool_.expand_pool(grid_size);
        RC_Vox_Bottom_Layer<data_type, MapInfo>::voxel_pool_.expand_pool(voxel_size);
    }

    void set_min_corner_pos(const double &min_corner_pos_x, const double &min_corner_pos_y, const double &min_corner_pos_z)
    {
        min_corner_pos_x_ = min_corner_pos_x;
        min_corner_pos_y_ = min_corner_pos_y;
        min_corner_pos_z_ = min_corner_pos_z;
    }

    MapInfo *find(const double &x, const double &y, const double &z)
    {
        pos_x_mod_tmp_ = (x - curr_corner_pos_x_) / grid_resolution_;
        pos_y_mod_tmp_ = (y - curr_corner_pos_y_) / grid_resolution_;
        pos_z_mod_tmp_ = (z - curr_corner_pos_z_) / grid_resolution_;

#ifdef SAFE_INSERT_AND_FIND
        if (pos_x_mod_tmp_ < boundary_x_min_ || pos_y_mod_tmp_ < boundary_y_min_ || pos_z_mod_tmp_ < boundary_z_min_ || pos_x_mod_tmp_ > boundary_x_max_ || pos_y_mod_tmp_ > boundary_y_max_ ||
            pos_z_mod_tmp_ > boundary_z_max_) {
            return nullptr;
        }
#endif

        if (pos_x_mod_tmp_ >= content_num_dim_x_) {
            pos_x_mod_tmp_ -= content_num_dim_x_;
        }
        if (pos_y_mod_tmp_ >= content_num_dim_y_) {
            pos_y_mod_tmp_ -= content_num_dim_y_;
        }
        if (pos_z_mod_tmp_ >= content_num_dim_z_) {
            pos_z_mod_tmp_ -= content_num_dim_z_;
        }

        key_tmp_ = content_num_dim_xy_ * pos_z_mod_tmp_ + content_num_dim_x_ * pos_y_mod_tmp_ + pos_x_mod_tmp_;

        if (map_content_[key_tmp_]) {
            return map_content_[key_tmp_]->find(x, y, z);
        } else {
            return nullptr;
        }
    }

    bool insert_point(const double &x, const double &y, const double &z)
    {
        pos_x_tmp_ = (x - curr_corner_pos_x_) / grid_resolution_;
        pos_y_tmp_ = (y - curr_corner_pos_y_) / grid_resolution_;
        pos_z_tmp_ = (z - curr_corner_pos_z_) / grid_resolution_;

#ifdef SAFE_INSERT_AND_FIND
        if (pos_x_tmp_ < boundary_x_min_ || pos_y_tmp_ < boundary_y_min_ || pos_z_tmp_ < boundary_z_min_ || pos_x_tmp_ > boundary_x_max_ || pos_y_tmp_ > boundary_y_max_ ||
            pos_z_tmp_ > boundary_z_max_) {
            return false;
        }
#endif

        pos_x_mod_tmp_ = pos_x_tmp_ >= content_num_dim_x_ ? pos_x_tmp_ - content_num_dim_x_ : pos_x_tmp_;
        pos_y_mod_tmp_ = pos_y_tmp_ >= content_num_dim_y_ ? pos_y_tmp_ - content_num_dim_y_ : pos_y_tmp_;
        pos_z_mod_tmp_ = pos_z_tmp_ >= content_num_dim_z_ ? pos_z_tmp_ - content_num_dim_z_ : pos_z_tmp_;

        key_tmp_ = content_num_dim_xy_ * pos_z_mod_tmp_ + content_num_dim_x_ * pos_y_mod_tmp_ + pos_x_mod_tmp_;

        if (!map_content_[key_tmp_]) {
#ifdef SAFE_ALLOCATE
            map_content_[key_tmp_] = grid_pool_.allocate_safe();
#else
            map_content_[key_tmp_] = grid_pool_.allocate_fast();
#endif
            map_content_[key_tmp_]->set_min_corner_pos(pos_x_tmp_ * grid_resolution_ + curr_corner_pos_x_, pos_y_tmp_ * grid_resolution_ + curr_corner_pos_y_,
                                                       pos_z_tmp_ * grid_resolution_ + curr_corner_pos_z_);
        }
        map_content_[key_tmp_]->insert_point(x, y, z);

        return true;
    }

    bool insert_point_auto_downsample(const double &x, const double &y, const double &z)
    {
        pos_x_tmp_ = (x - curr_corner_pos_x_) / grid_resolution_;
        pos_y_tmp_ = (y - curr_corner_pos_y_) / grid_resolution_;
        pos_z_tmp_ = (z - curr_corner_pos_z_) / grid_resolution_;

#ifdef SAFE_INSERT_AND_FIND
        if (pos_x_tmp_ < boundary_x_min_ || pos_y_tmp_ < boundary_y_min_ || pos_z_tmp_ < boundary_z_min_ || pos_x_tmp_ > boundary_x_max_ || pos_y_tmp_ > boundary_y_max_ ||
            pos_z_tmp_ > boundary_z_max_) {
            return false;
        }
#endif

        pos_x_mod_tmp_ = pos_x_tmp_ >= content_num_dim_x_ ? pos_x_tmp_ - content_num_dim_x_ : pos_x_tmp_;
        pos_y_mod_tmp_ = pos_y_tmp_ >= content_num_dim_y_ ? pos_y_tmp_ - content_num_dim_y_ : pos_y_tmp_;
        pos_z_mod_tmp_ = pos_z_tmp_ >= content_num_dim_z_ ? pos_z_tmp_ - content_num_dim_z_ : pos_z_tmp_;

        key_tmp_ = content_num_dim_xy_ * pos_z_mod_tmp_ + content_num_dim_x_ * pos_y_mod_tmp_ + pos_x_mod_tmp_;

        if (!map_content_[key_tmp_]) {
#ifdef SAFE_ALLOCATE
            map_content_[key_tmp_] = grid_pool_.allocate_safe();
#else
            map_content_[key_tmp_] = grid_pool_.allocate_fast();
#endif
            map_content_[key_tmp_]->set_min_corner_pos(pos_x_tmp_ * grid_resolution_ + curr_corner_pos_x_, pos_y_tmp_ * grid_resolution_ + curr_corner_pos_y_,
                                                       pos_z_tmp_ * grid_resolution_ + curr_corner_pos_z_);
        }
        return map_content_[key_tmp_]->insert_point_auto_downsample(x, y, z);
    }

    // 前提是调用了 insert_point
    void insert_point_to_nearby_point_seed(const double &x, const double &y, const double &z, const double &x_info, const double &y_info, const double &z_info)
    {
        pos_x_tmp_ = (x - curr_corner_pos_x_) / grid_resolution_;
        pos_y_tmp_ = (y - curr_corner_pos_y_) / grid_resolution_;
        pos_z_tmp_ = (z - curr_corner_pos_z_) / grid_resolution_;

        // #ifdef SAFE_INSERT_AND_FIND
        //     if(pos_x_tmp_ < boundary_x_min_ || pos_y_tmp_ < boundary_y_min_ || pos_z_tmp_ < boundary_z_min_
        //         || pos_x_tmp_ > boundary_x_max_ || pos_y_tmp_ > boundary_y_max_ || pos_z_tmp_ > boundary_z_max_) {
        //         return;
        //     }
        // #endif

        pos_x_mod_tmp_ = pos_x_tmp_ >= content_num_dim_x_ ? pos_x_tmp_ - content_num_dim_x_ : pos_x_tmp_;
        pos_y_mod_tmp_ = pos_y_tmp_ >= content_num_dim_y_ ? pos_y_tmp_ - content_num_dim_y_ : pos_y_tmp_;
        pos_z_mod_tmp_ = pos_z_tmp_ >= content_num_dim_z_ ? pos_z_tmp_ - content_num_dim_z_ : pos_z_tmp_;

        key_tmp_ = content_num_dim_xy_ * pos_z_mod_tmp_ + content_num_dim_x_ * pos_y_mod_tmp_ + pos_x_mod_tmp_;

        if (!map_content_[key_tmp_]) {
#ifdef SAFE_ALLOCATE
            map_content_[key_tmp_] = grid_pool_.allocate_safe();
#else
            map_content_[key_tmp_] = grid_pool_.allocate_fast();
#endif
            map_content_[key_tmp_]->set_min_corner_pos(pos_x_tmp_ * grid_resolution_ + curr_corner_pos_x_, pos_y_tmp_ * grid_resolution_ + curr_corner_pos_y_,
                                                       pos_z_tmp_ * grid_resolution_ + curr_corner_pos_z_);
        }
        map_content_[key_tmp_]->insert_point_to_nearby_point_seed(x, y, z, x_info, y_info, z_info);
    }

    // 前提是调用了 insert_point
    void insert_point_to_nearby_voxel(const double &x, const double &y, const double &z, const double &x_info, const double &y_info, const double &z_info)
    {
        pos_x_tmp_ = (x - curr_corner_pos_x_) / grid_resolution_;
        pos_y_tmp_ = (y - curr_corner_pos_y_) / grid_resolution_;
        pos_z_tmp_ = (z - curr_corner_pos_z_) / grid_resolution_;

        // #ifdef SAFE_INSERT_AND_FIND
        //     if(pos_x_tmp_ < boundary_x_min_ || pos_y_tmp_ < boundary_y_min_ || pos_z_tmp_ < boundary_z_min_
        //         || pos_x_tmp_ > boundary_x_max_ || pos_y_tmp_ > boundary_y_max_ || pos_z_tmp_ > boundary_z_max_) {
        //         return;
        //     }
        // #endif

        pos_x_mod_tmp_ = pos_x_tmp_ >= content_num_dim_x_ ? pos_x_tmp_ - content_num_dim_x_ : pos_x_tmp_;
        pos_y_mod_tmp_ = pos_y_tmp_ >= content_num_dim_y_ ? pos_y_tmp_ - content_num_dim_y_ : pos_y_tmp_;
        pos_z_mod_tmp_ = pos_z_tmp_ >= content_num_dim_z_ ? pos_z_tmp_ - content_num_dim_z_ : pos_z_tmp_;

        key_tmp_ = content_num_dim_xy_ * pos_z_mod_tmp_ + content_num_dim_x_ * pos_y_mod_tmp_ + pos_x_mod_tmp_;

        if (!map_content_[key_tmp_]) {
#ifdef SAFE_ALLOCATE
            map_content_[key_tmp_] = grid_pool_.allocate_safe();
#else
            map_content_[key_tmp_] = grid_pool_.allocate_fast();
#endif
            map_content_[key_tmp_]->set_min_corner_pos(pos_x_tmp_ * grid_resolution_ + curr_corner_pos_x_, pos_y_tmp_ * grid_resolution_ + curr_corner_pos_y_,
                                                       pos_z_tmp_ * grid_resolution_ + curr_corner_pos_z_);
        }
        map_content_[key_tmp_]->insert_point_to_nearby_voxel(x, y, z, x_info, y_info, z_info);
    }

    void incremental_cut_map(const double &x, const double &y, const double &z)
    { // 输入机器人当前位置
        curr_robot_position_x_ = x;
        curr_robot_position_y_ = y;
        curr_robot_position_z_ = z;

        if (flg_need_init_robot_position_) { // 机器人开始在地图中心位置
            set_min_corner_pos(x - content_num_dim_x_ * grid_resolution_ / 2.0, y - content_num_dim_y_ * grid_resolution_ / 2.0, z - content_num_dim_z_ * grid_resolution_ / 2.0);
            flg_need_init_robot_position_ = false;

            last_update_robot_position_x_ = data_type(std::floor((x - min_corner_pos_x_) / grid_resolution_));
            last_update_robot_position_y_ = data_type(std::floor((y - min_corner_pos_y_) / grid_resolution_));
            last_update_robot_position_z_ = data_type(std::floor((z - min_corner_pos_z_) / grid_resolution_));

            begin_2_center_length_x_ = last_update_robot_position_x_;
            begin_2_center_length_y_ = last_update_robot_position_y_;
            begin_2_center_length_z_ = last_update_robot_position_z_;
            center_2_end_length_x_ = content_num_dim_x_ - 1 - last_update_robot_position_x_;
            center_2_end_length_y_ = content_num_dim_y_ - 1 - last_update_robot_position_y_;
            center_2_end_length_z_ = content_num_dim_z_ - 1 - last_update_robot_position_z_;

            data_type curr_corner_offset_x_ = data_type(std::floor((x - content_num_dim_x_ * grid_resolution_ / 2.0 - min_corner_pos_x_) / grid_resolution_)) % content_num_dim_x_;
            curr_corner_offset_x_ = curr_corner_offset_x_ < 0 ? curr_corner_offset_x_ + content_num_dim_x_ : curr_corner_offset_x_;
            data_type curr_corner_offset_y_ = data_type(std::floor((y - content_num_dim_y_ * grid_resolution_ / 2.0 - min_corner_pos_y_) / grid_resolution_)) % content_num_dim_y_;
            curr_corner_offset_y_ = curr_corner_offset_y_ < 0 ? curr_corner_offset_y_ + content_num_dim_y_ : curr_corner_offset_y_;
            data_type curr_corner_offset_z_ = data_type(std::floor((z - content_num_dim_z_ * grid_resolution_ / 2.0 - min_corner_pos_z_) / grid_resolution_)) % content_num_dim_z_;
            curr_corner_offset_z_ = curr_corner_offset_z_ < 0 ? curr_corner_offset_z_ + content_num_dim_z_ : curr_corner_offset_z_;
            curr_corner_pos_x_ =
                std::floor((x - content_num_dim_x_ * grid_resolution_ / 2.0 - min_corner_pos_x_) / grid_resolution_) * grid_resolution_ + min_corner_pos_x_ - curr_corner_offset_x_ * grid_resolution_;
            curr_corner_pos_y_ =
                std::floor((y - content_num_dim_y_ * grid_resolution_ / 2.0 - min_corner_pos_y_) / grid_resolution_) * grid_resolution_ + min_corner_pos_y_ - curr_corner_offset_y_ * grid_resolution_;
            curr_corner_pos_z_ =
                std::floor((z - content_num_dim_z_ * grid_resolution_ / 2.0 - min_corner_pos_z_) / grid_resolution_) * grid_resolution_ + min_corner_pos_z_ - curr_corner_offset_z_ * grid_resolution_;
            boundary_x_min_ = curr_corner_offset_x_ + 2;
            boundary_y_min_ = curr_corner_offset_y_ + 2;
            boundary_z_min_ = curr_corner_offset_z_ + 2;
            boundary_x_max_ = curr_corner_offset_x_ + content_num_dim_x_ - 3;
            boundary_y_max_ = curr_corner_offset_y_ + content_num_dim_y_ - 3;
            boundary_z_max_ = curr_corner_offset_z_ + content_num_dim_z_ - 3;

            return;
        } else {
            // x - content_num_dim_x_ * grid_resolution_ / 2.0 : 当前位置下 localmap 左下角角点位置 x 分量的 map 坐标(栅格)
            // curr_corner_offset_x_ : 当前位置下 submap 左下角角点位置 x 分量的 localmap 坐标(栅格)
            // curr_corner_pos_x_ : 当前位置下 localmap 左下角角点位置 x 分量的 map 坐标
            data_type curr_corner_offset_x_ = data_type(std::floor((x - content_num_dim_x_ * grid_resolution_ / 2.0 - min_corner_pos_x_) / grid_resolution_)) % content_num_dim_x_;
            curr_corner_offset_x_ = curr_corner_offset_x_ < 0 ? curr_corner_offset_x_ + content_num_dim_x_ : curr_corner_offset_x_;
            data_type curr_corner_offset_y_ = data_type(std::floor((y - content_num_dim_y_ * grid_resolution_ / 2.0 - min_corner_pos_y_) / grid_resolution_)) % content_num_dim_y_;
            curr_corner_offset_y_ = curr_corner_offset_y_ < 0 ? curr_corner_offset_y_ + content_num_dim_y_ : curr_corner_offset_y_;
            data_type curr_corner_offset_z_ = data_type(std::floor((z - content_num_dim_z_ * grid_resolution_ / 2.0 - min_corner_pos_z_) / grid_resolution_)) % content_num_dim_z_;
            curr_corner_offset_z_ = curr_corner_offset_z_ < 0 ? curr_corner_offset_z_ + content_num_dim_z_ : curr_corner_offset_z_;
            curr_corner_pos_x_ =
                std::floor((x - content_num_dim_x_ * grid_resolution_ / 2.0 - min_corner_pos_x_) / grid_resolution_) * grid_resolution_ + min_corner_pos_x_ - curr_corner_offset_x_ * grid_resolution_;
            curr_corner_pos_y_ =
                std::floor((y - content_num_dim_y_ * grid_resolution_ / 2.0 - min_corner_pos_y_) / grid_resolution_) * grid_resolution_ + min_corner_pos_y_ - curr_corner_offset_y_ * grid_resolution_;
            curr_corner_pos_z_ =
                std::floor((z - content_num_dim_z_ * grid_resolution_ / 2.0 - min_corner_pos_z_) / grid_resolution_) * grid_resolution_ + min_corner_pos_z_ - curr_corner_offset_z_ * grid_resolution_;
            boundary_x_min_ = curr_corner_offset_x_ + 2;
            boundary_y_min_ = curr_corner_offset_y_ + 2;
            boundary_z_min_ = curr_corner_offset_z_ + 2;
            boundary_x_max_ = curr_corner_offset_x_ + content_num_dim_x_ - 3;
            boundary_y_max_ = curr_corner_offset_y_ + content_num_dim_y_ - 3;
            boundary_z_max_ = curr_corner_offset_z_ + content_num_dim_z_ - 3;

            pos_x_tmp_ = data_type(std::floor((x - min_corner_pos_x_) / grid_resolution_));
            pos_y_tmp_ = data_type(std::floor((y - min_corner_pos_y_) / grid_resolution_));
            pos_z_tmp_ = data_type(std::floor((z - min_corner_pos_z_) / grid_resolution_));

            if (pos_x_tmp_ != last_update_robot_position_x_) {
                if (pos_x_tmp_ > last_update_robot_position_x_) {
                    int x_tmp;
                    for (int x = last_update_robot_position_x_ - begin_2_center_length_x_; x < pos_x_tmp_ - begin_2_center_length_x_; x++) {
                        x_tmp = x % content_num_dim_x_;
                        x_tmp = ((x_tmp < 0) ? (x_tmp + content_num_dim_x_) : x_tmp);
                        for (int y = 0; y < content_num_dim_y_; y++) {
                            for (int z = 0; z < content_num_dim_z_; z++) {
                                key_tmp_ = content_num_dim_xy_ * z + content_num_dim_x_ * y + x_tmp;
                                if (map_content_[key_tmp_]) {
                                    grid_pool_.deallocate(map_content_[key_tmp_]);
                                    map_content_[key_tmp_] = nullptr;
                                }
                            }
                        }
                    }
                } else {
                    int x_tmp;
                    for (int x = last_update_robot_position_x_ + center_2_end_length_x_; x > pos_x_tmp_ + center_2_end_length_x_; x--) {
                        x_tmp = x % content_num_dim_x_;
                        x_tmp = ((x_tmp < 0) ? (x_tmp + content_num_dim_x_) : x_tmp);
                        for (int y = 0; y < content_num_dim_y_; y++) {
                            for (int z = 0; z < content_num_dim_z_; z++) {
                                key_tmp_ = content_num_dim_xy_ * z + content_num_dim_x_ * y + x_tmp;
                                if (map_content_[key_tmp_]) {
                                    grid_pool_.deallocate(map_content_[key_tmp_]);
                                    map_content_[key_tmp_] = nullptr;
                                }
                            }
                        }
                    }
                }
            }

            if (pos_y_tmp_ != last_update_robot_position_y_) {
                if (pos_y_tmp_ > last_update_robot_position_y_) {
                    int y_tmp;
                    for (int y = last_update_robot_position_y_ - begin_2_center_length_y_; y < pos_y_tmp_ - begin_2_center_length_y_; y++) {
                        y_tmp = y % content_num_dim_y_;
                        y_tmp = ((y_tmp < 0) ? (y_tmp + content_num_dim_y_) : y_tmp);
                        for (int x = 0; x < content_num_dim_x_; x++) {
                            for (int z = 0; z < content_num_dim_z_; z++) {
                                key_tmp_ = content_num_dim_xy_ * z + content_num_dim_x_ * y_tmp + x;
                                if (map_content_[key_tmp_]) {
                                    grid_pool_.deallocate(map_content_[key_tmp_]);
                                    map_content_[key_tmp_] = nullptr;
                                }
                            }
                        }
                    }
                } else {
                    int y_tmp;
                    for (int y = last_update_robot_position_y_ + center_2_end_length_y_; y > pos_y_tmp_ + center_2_end_length_y_; y--) {
                        y_tmp = y % content_num_dim_y_;
                        y_tmp = ((y_tmp < 0) ? (y_tmp + content_num_dim_y_) : y_tmp);
                        for (int x = 0; x < content_num_dim_x_; x++) {
                            for (int z = 0; z < content_num_dim_z_; z++) {
                                key_tmp_ = content_num_dim_xy_ * z + content_num_dim_x_ * y_tmp + x;
                                if (map_content_[key_tmp_]) {
                                    grid_pool_.deallocate(map_content_[key_tmp_]);
                                    map_content_[key_tmp_] = nullptr;
                                }
                            }
                        }
                    }
                }
            }
            if (pos_z_tmp_ != last_update_robot_position_z_) {
                if (pos_z_tmp_ > last_update_robot_position_z_) {
                    int z_tmp;
                    for (int z = last_update_robot_position_z_ - begin_2_center_length_z_; z < pos_z_tmp_ - begin_2_center_length_z_; z++) {
                        z_tmp = z % content_num_dim_z_;
                        z_tmp = ((z_tmp < 0) ? (z_tmp + content_num_dim_z_) : z_tmp);
                        for (int y = 0; y < content_num_dim_y_; y++) {
                            for (int x = 0; x < content_num_dim_x_; x++) {
                                key_tmp_ = content_num_dim_xy_ * z_tmp + content_num_dim_x_ * y + x;
                                if (map_content_[key_tmp_]) {
                                    grid_pool_.deallocate(map_content_[key_tmp_]);
                                    map_content_[key_tmp_] = nullptr;
                                }
                            }
                        }
                    }
                } else {
                    int z_tmp;
                    for (int z = last_update_robot_position_z_ + center_2_end_length_z_; z > pos_z_tmp_ + center_2_end_length_z_; z--) {
                        z_tmp = z % content_num_dim_z_;
                        z_tmp = ((z_tmp < 0) ? (z_tmp + content_num_dim_z_) : z_tmp);
                        for (int y = 0; y < content_num_dim_y_; y++) {
                            for (int x = 0; x < content_num_dim_x_; x++) {
                                key_tmp_ = content_num_dim_xy_ * z_tmp + content_num_dim_x_ * y + x;
                                if (map_content_[key_tmp_]) {
                                    grid_pool_.deallocate(map_content_[key_tmp_]);
                                    map_content_[key_tmp_] = nullptr;
                                }
                            }
                        }
                    }
                }
            }
            last_update_robot_position_x_ = pos_x_tmp_;
            last_update_robot_position_y_ = pos_y_tmp_;
            last_update_robot_position_z_ = pos_z_tmp_;

            return;
        }
    }

    void get_all_pts(std::vector<MapInfo *> &pts)
    {
        for (auto &grid_ptr : map_content_) {
            if (grid_ptr) {
                grid_ptr->get_all_pts(pts);
            }
        }
    }

    double get_grid_resolution()
    {
        return grid_resolution_;
    }

    data_type get_range_x()
    {
        return content_num_dim_x_;
    }
    data_type get_range_y()
    {
        return content_num_dim_y_;
    }
    data_type get_range_z()
    {
        return content_num_dim_z_;
    }

    double get_curr_robot_position_x()
    {
        return curr_robot_position_x_;
    }
    double get_curr_robot_position_y()
    {
        return curr_robot_position_y_;
    }
    double get_curr_robot_position_z()
    {
        return curr_robot_position_z_;
    }

    Eigen::Vector3d get_min_corner_pos()
    {
        return Eigen::Vector3d(min_corner_pos_x_, min_corner_pos_y_, min_corner_pos_z_);
    }

public:
    Custom_Memory_Pool<RC_Vox_Bottom_Layer<data_type, MapInfo>> grid_pool_;
    std::vector<RC_Vox_Bottom_Layer<data_type, MapInfo> *> map_content_;

    data_type content_num_dim_x_;
    data_type content_num_dim_y_;
    data_type content_num_dim_z_;
    data_type content_num_dim_xy_;

    double min_corner_pos_x_;
    double min_corner_pos_y_;
    double min_corner_pos_z_;

    double grid_resolution_;
    double voxel_resolution_;
    double point_resolution_;

    bool flg_need_init_robot_position_;
    data_type last_update_robot_position_x_;
    data_type last_update_robot_position_y_;
    data_type last_update_robot_position_z_;

    data_type begin_2_center_length_x_;
    data_type begin_2_center_length_y_;
    data_type begin_2_center_length_z_;
    data_type center_2_end_length_x_, center_2_end_length_y_, center_2_end_length_z_;

    double curr_corner_pos_x_, curr_corner_pos_y_, curr_corner_pos_z_;
    double curr_robot_position_x_, curr_robot_position_y_, curr_robot_position_z_;
    data_type boundary_x_min_, boundary_y_min_, boundary_z_min_;
    data_type boundary_x_max_, boundary_y_max_, boundary_z_max_;

    data_type key_tmp_;
    data_type pos_x_tmp_, pos_y_tmp_, pos_z_tmp_;
    data_type pos_x_mod_tmp_, pos_y_mod_tmp_, pos_z_mod_tmp_;
};