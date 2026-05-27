#pragma once
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

#include <iostream>
#include <algorithm>
#include <glog/logging.h>

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

/**
 * @brief LiDAR消息格式统一处理模块。
 *
 * 目标：
 * 1. 兼容多种数据集与雷达厂商点类型。
 * 2. 统一转换到内部点类型 PointXYZIRCT。
 * 3. 给出每帧起止时间与有效点数量，供后续里程计使用。
 */

namespace Baidu
{
struct PointXYZIRCT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY_8U;
    uint16_t row;
    uint16_t column;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(Baidu::PointXYZIRCT,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, row, row)(uint16_t, column, column)(double, timestamp, timestamp))

namespace RoboSense
{
struct PointXYZIRT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(RoboSense::PointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

namespace Velodyne
{
struct PointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(Velodyne::PointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

namespace Velodyne
{
struct PointXYZRT {
    PCL_ADD_POINT4D;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(Velodyne::PointXYZRT, (float, x, x)(float, y, y)(float, z, z)(uint16_t, ring, ring)(float, time, time))

namespace Velodyne
{
struct PointXYZIR {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(Velodyne::PointXYZIR, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

// namespace Hesai {
// 	struct PointXYZITR {
// 		PCL_ADD_POINT4D;
// 		PCL_ADD_INTENSITY;
// 		double timestamp;
// 		uint16_t ring;
// 		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// 	} EIGEN_ALIGN16;
// }
// POINT_CLOUD_REGISTER_POINT_STRUCT (Hesai::PointXYZITR,
// 	(float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
// 	(double, timestamp, timestamp) (uint16_t, ring, ring)
// )
namespace Hesai
{
struct PointXYZITR {
    PCL_ADD_POINT4D //添加pcl里xyz
        float intensity;
    double timestamp;
    uint16_t ring; ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned,确保定义新类型点云内存与SSE对齐
} EIGEN_ALIGN16; // 强制SSE填充以正确对齐内存
}
POINT_CLOUD_REGISTER_POINT_STRUCT(Hesai::PointXYZITR, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

namespace Ouster
{
struct PointXYZITRRNR {
    PCL_ADD_POINT4D //添加pcl里xyz
        float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range; ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned,确保定义新类型点云内存与SSE对齐
} EIGEN_ALIGN16; // 强制SSE填充以正确对齐内存
}
POINT_CLOUD_REGISTER_POINT_STRUCT(Ouster::PointXYZITRRNR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint8_t, ring, ring)(uint32_t, range,
                                                                                                                                                                                  range))

struct PointXYZIRCT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY_8U;
    uint16_t label;
    uint16_t row;
    uint16_t column;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRCT, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity,
                                                                                        intensity)(uint16_t, label, label)(uint16_t, row, row)(uint16_t, column, column)(double, timestamp, timestamp))

class Lidar_Format_Processor {
public:
    /**
     * @brief 标准化输出数据。
     */
    struct Data {
        pcl::PointCloud<PointXYZIRCT>::Ptr cloud;
        double begin_time;
        double end_time;
        int valid_pt_num;
    };

    /**
     * @brief 支持的LiDAR数据格式枚举。
     */
    enum LIDAR_TYPE {
        LIDAR_TYPE_baidu_hesai_Pandar40P = 1,
        LIDAR_TYPE_baidu_velodyne_HDL32E = 2,
        LIDAR_TYPE_labs_robosense_RSLiDAR16 = 3,
        LIDAR_TYPE_labs_velodyne_VLP16 = 4,
        LIDAR_TYPE_urbannav_velodyne_HDL32E = 5,
        LIDAR_TYPE_urbanloco_robosense_RSLiDAR32 = 6,
        LIDAR_TYPE_urbannav_new_velodyne_HDL32E = 7,
        LIDAR_TYPE_hilti_hesai_PandarXT32 = 8,
        LIDAR_TYPE_labs_livox = 9,
        LIDAR_TYPE_newercollege_ouster_OS1 = 10,
        LIDAR_TYPE_ntuviral_ouster_OS1 = 11,
        LIDAR_TYPE_urbanloco_velodyne_HDL32E = 12,
        LIDAR_TYPE_nclt_new_velodyne_HDL32E = 13,
    };

    Lidar_Format_Processor(){};
    Lidar_Format_Processor(LIDAR_TYPE lidar_type, int lidar_image_rows, int lidar_image_cols);

    // 从配置文件读取雷达类型并完成内部参数初始化。
    void load_config_parameter(const std::string &sensor_config_file);
    void set_config_parameter(std::string param_lidar_type);
    void init_param(LIDAR_TYPE lidar_type, int lidar_image_rows, int lidar_image_cols);
    // 目前仅百度数据中的LiDAR支持此功能
    void enable_parameter_auto_recognize(LIDAR_TYPE lidar_type);
    Data process(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());

    LIDAR_TYPE get_param_lidar_type()
    {
        return param_lidar_type_;
    }

protected:
    Data baidu_hesai_Pandar40P_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());
    Data baidu_velodyne_HDL32E_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());
    Data labs_robosense_RSLiDAR16_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());
    Data labs_velodyne_VLP16_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());
    Data urbannav_velodyne_HDL32E_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());
    Data urbannav_new_velodyne_HDL32E_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());
    Data nclt_velodyne_HDL32E_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());
    Data urbanloco_velodyne_HDL32E_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());
    Data hilti_hesai_PandarXT32_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());
    Data newercollege_ouster_OS1_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());
    Data ntuviral_ouster_OS1_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());
    Data labs_livox_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string = std::string());
    LIDAR_TYPE param_lidar_type_;
    int param_lidar_image_rows_;
    int param_lidar_image_cols_;
    int param_parameter_auto_recognize_ = false;
};

Lidar_Format_Processor::Lidar_Format_Processor(LIDAR_TYPE lidar_type, int lidar_image_rows, int lidar_image_cols)
{
    init_param(lidar_type, lidar_image_rows, lidar_image_cols);
}

void Lidar_Format_Processor::load_config_parameter(const std::string &sensor_config_file)
{
    YAML::Node config_node = YAML::LoadFile(sensor_config_file);

    std::string param_lidar_type;
    try {
        param_lidar_type = config_node["param_lidar_type"].as<std::string>();
    } catch (...) {
        LOG(FATAL) << "[Imu_Format_Processor]: file " << sensor_config_file << " has a bad conversion";
    }

    LOG(INFO) << std::endl << "| --------------- [PointCloud_Map parameter] --------------- |" << std::endl << "[param_lidar_type] " << param_lidar_type << std::endl << std::endl;

    set_config_parameter(param_lidar_type);
}

void Lidar_Format_Processor::set_config_parameter(std::string param_lidar_type)
{
    if (param_lidar_type == "LIDAR_TYPE_baidu_hesai_Pandar40P") {
        param_lidar_type_ = LIDAR_TYPE_baidu_hesai_Pandar40P;
    } else if (param_lidar_type == "LIDAR_TYPE_baidu_velodyne_HDL32E") {
        param_lidar_type_ = LIDAR_TYPE_baidu_velodyne_HDL32E;
    } else if (param_lidar_type == "LIDAR_TYPE_labs_robosense_RSLiDAR16") {
        param_lidar_type_ = LIDAR_TYPE_labs_robosense_RSLiDAR16;
    } else if (param_lidar_type == "LIDAR_TYPE_labs_velodyne_VLP16") {
        param_lidar_type_ = LIDAR_TYPE_labs_velodyne_VLP16;
    } else if (param_lidar_type == "LIDAR_TYPE_urbannav_velodyne_HDL32E") {
        param_lidar_type_ = LIDAR_TYPE_urbannav_velodyne_HDL32E;
    } else if (param_lidar_type == "LIDAR_TYPE_urbannav_new_velodyne_HDL32E") {
        param_lidar_type_ = LIDAR_TYPE_urbannav_new_velodyne_HDL32E;
    } else if (param_lidar_type == "LIDAR_TYPE_hilti_hesai_PandarXT32") {
        param_lidar_type_ = LIDAR_TYPE_hilti_hesai_PandarXT32;
    } else if (param_lidar_type == "LIDAR_TYPE_labs_livox") {
        param_lidar_type_ = LIDAR_TYPE_labs_livox;
    } else if (param_lidar_type == "LIDAR_TYPE_newercollege_ouster_OS1") {
        param_lidar_type_ = LIDAR_TYPE_newercollege_ouster_OS1;
    } else if (param_lidar_type == "LIDAR_TYPE_ntuviral_ouster_OS1") {
        param_lidar_type_ = LIDAR_TYPE_ntuviral_ouster_OS1;
    } else if (param_lidar_type == "LIDAR_TYPE_urbanloco_velodyne_HDL32E") {
        param_lidar_type_ = LIDAR_TYPE_urbanloco_velodyne_HDL32E;
    } else if (param_lidar_type == "LIDAR_TYPE_nclt_new_velodyne_HDL32E") {
        param_lidar_type_ = LIDAR_TYPE_nclt_new_velodyne_HDL32E;
    }
}

void Lidar_Format_Processor::init_param(LIDAR_TYPE lidar_type, int lidar_image_rows, int lidar_image_cols)
{
    param_lidar_type_ = lidar_type;
    param_lidar_image_rows_ = lidar_image_rows;
    param_lidar_image_cols_ = lidar_image_cols;
    param_parameter_auto_recognize_ = false;
}

void Lidar_Format_Processor::enable_parameter_auto_recognize(LIDAR_TYPE lidar_type)
{
    param_lidar_type_ = lidar_type;
    param_parameter_auto_recognize_ = true;
}

Lidar_Format_Processor::Data Lidar_Format_Processor::process(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    switch (param_lidar_type_) {
    case LIDAR_TYPE_baidu_hesai_Pandar40P:
        return baidu_hesai_Pandar40P_handler(timestamp, msg, label_string);
        break;
    case LIDAR_TYPE_baidu_velodyne_HDL32E:
        return baidu_velodyne_HDL32E_handler(timestamp, msg, label_string);
        break;
    case LIDAR_TYPE_labs_robosense_RSLiDAR16:
        return labs_robosense_RSLiDAR16_handler(timestamp, msg, label_string);
        break;
    case LIDAR_TYPE_labs_velodyne_VLP16:
        return labs_velodyne_VLP16_handler(timestamp, msg, label_string);
        break;
    case LIDAR_TYPE_urbannav_velodyne_HDL32E:
        return urbannav_velodyne_HDL32E_handler(timestamp, msg, label_string);
        break;
    case LIDAR_TYPE_urbannav_new_velodyne_HDL32E:
        return urbannav_new_velodyne_HDL32E_handler(timestamp, msg, label_string);
        break;
    case LIDAR_TYPE_nclt_new_velodyne_HDL32E:
        return nclt_velodyne_HDL32E_handler(timestamp, msg, label_string);
        break;
    case LIDAR_TYPE_urbanloco_velodyne_HDL32E:
        return urbanloco_velodyne_HDL32E_handler(timestamp, msg, label_string);
        break;
    case LIDAR_TYPE_hilti_hesai_PandarXT32:
        return hilti_hesai_PandarXT32_handler(timestamp, msg, label_string);
        break;
    case LIDAR_TYPE_labs_livox:
        return labs_livox_handler(timestamp, msg, label_string);
        break;
    case LIDAR_TYPE_newercollege_ouster_OS1:
        return newercollege_ouster_OS1_handler(timestamp, msg, label_string);
        break;
    case LIDAR_TYPE_ntuviral_ouster_OS1:
        return ntuviral_ouster_OS1_handler(timestamp, msg, label_string);
        break;
    default:
        LOG(FATAL) << "未定义的点云类型" << std::endl;
    }
}

Lidar_Format_Processor::Data Lidar_Format_Processor::baidu_hesai_Pandar40P_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    pcl::PointCloud<Baidu::PointXYZIRCT> raw_cloud;
    pcl::fromPCLPointCloud2(*msg, raw_cloud);

    int plsize = raw_cloud.size();
    int max_width = 0, max_height = 0;
    double min_time = DBL_MAX, max_time = DBL_MIN;
    for (uint i = 0; i < plsize; i++) {
        if (max_width < raw_cloud.points[i].column) {
            max_width = raw_cloud.points[i].column;
        }
        if (max_height < raw_cloud.points[i].row) {
            max_height = raw_cloud.points[i].row;
        }
        if (min_time > raw_cloud.points[i].timestamp) {
            min_time = raw_cloud.points[i].timestamp;
        }
        if (max_time < raw_cloud.points[i].timestamp) {
            max_time = raw_cloud.points[i].timestamp;
        }
    }

    if (param_parameter_auto_recognize_) {
        param_lidar_image_rows_ = max_height + 1;
        param_lidar_image_cols_ = max_width + 1;
    }

    std::vector<std::vector<int>> idx_mat(param_lidar_image_rows_, std::vector<int>(param_lidar_image_cols_, -1));
    for (uint i = 0; i < plsize; i++) {
        idx_mat[raw_cloud.points[i].row][raw_cloud.points[i].column] = i;
    }

    pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    if (label_string.length() != 0) {
        PointXYZIRCT added_pt;
        for (uint i = 0; i < idx_mat.size(); i++) {
            for (uint j = 0; j < idx_mat[i].size(); j++) {
                if (idx_mat[i][j] != -1) {
                    std::size_t idx = idx_mat[i][j];
                    added_pt.x = raw_cloud.points[idx].x;
                    added_pt.y = raw_cloud.points[idx].y;
                    added_pt.z = raw_cloud.points[idx].z;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = raw_cloud.points[idx].timestamp /* -min_time */;
                    added_pt.label = /* 0 */ label_string[i * idx_mat[i].size() + j] - '0';
                    cloud->push_back(added_pt);
                } else {
                    added_pt.x = 10000.0;
                    added_pt.y = 10000.0;
                    added_pt.z = 10000.0;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = -10;
                    added_pt.label = 1000; // 无效点
                    cloud->push_back(added_pt);
                }
            }
        }
    } else {
        PointXYZIRCT added_pt;
        for (uint i = 0; i < idx_mat.size(); i++) {
            for (uint j = 0; j < idx_mat[i].size(); j++) {
                if (idx_mat[i][j] != -1) {
                    std::size_t idx = idx_mat[i][j];
                    added_pt.x = raw_cloud.points[idx].x;
                    added_pt.y = raw_cloud.points[idx].y;
                    added_pt.z = raw_cloud.points[idx].z;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = raw_cloud.points[idx].timestamp /* -min_time */;
                    added_pt.label = 0;
                    cloud->push_back(added_pt);
                } else {
                    added_pt.x = 10000.0;
                    added_pt.y = 10000.0;
                    added_pt.z = 10000.0;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = -10;
                    added_pt.label = 1000; // 无效点
                    cloud->push_back(added_pt);
                }
            }
        }
    }

    Lidar_Format_Processor::Data lidar_data;
    lidar_data.cloud = cloud;
    lidar_data.begin_time = min_time;
    lidar_data.end_time = max_time;
    lidar_data.valid_pt_num = plsize;
    return lidar_data;
}

Lidar_Format_Processor::Data Lidar_Format_Processor::baidu_velodyne_HDL32E_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    pcl::PointCloud<Baidu::PointXYZIRCT> raw_cloud;
    pcl::fromPCLPointCloud2(*msg, raw_cloud);

    int plsize = raw_cloud.size(), valid_size = 0;
    int max_width = 0, max_height = 0;
    double min_time = DBL_MAX, max_time = DBL_MIN;
    for (uint i = 0; i < plsize; i++) {
        if (raw_cloud.points[i].timestamp == 0) {
            continue;
        }
        valid_size++;
        if (max_width < raw_cloud.points[i].column) {
            max_width = raw_cloud.points[i].column;
        }
        if (max_height < raw_cloud.points[i].row) {
            max_height = raw_cloud.points[i].row;
        }
        if (min_time > raw_cloud.points[i].timestamp) {
            min_time = raw_cloud.points[i].timestamp;
        }
        if (max_time < raw_cloud.points[i].timestamp) {
            max_time = raw_cloud.points[i].timestamp;
        }
    }

    if (param_parameter_auto_recognize_) {
        param_lidar_image_rows_ = max_height + 1;
        param_lidar_image_cols_ = max_width + 1;
    }

    std::vector<std::vector<int>> idx_mat(param_lidar_image_rows_, std::vector<int>(param_lidar_image_cols_, -1));
    for (uint i = 0; i < plsize; i++) {
        idx_mat[raw_cloud.points[i].row][raw_cloud.points[i].column] = i;
    }

    pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    if (label_string.length() != 0) {
        PointXYZIRCT added_pt;
        for (uint i = 0; i < idx_mat.size(); i++) {
            for (uint j = 0; j < idx_mat[i].size(); j++) {
                if (idx_mat[i][j] != -1) {
                    std::size_t idx = idx_mat[i][j];
                    added_pt.x = raw_cloud.points[idx].x;
                    added_pt.y = raw_cloud.points[idx].y;
                    added_pt.z = raw_cloud.points[idx].z;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = raw_cloud.points[idx].timestamp /* -min_time */;
                    added_pt.label = /* 0 */ label_string[i * idx_mat[i].size() + j] - '0';
                    cloud->push_back(added_pt);
                } else {
                    added_pt.x = 10000.0;
                    added_pt.y = 10000.0;
                    added_pt.z = 10000.0;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = -10;
                    added_pt.label = 1000; // 无效点
                    cloud->push_back(added_pt);
                }
            }
        }
    } else {
        PointXYZIRCT added_pt;
        for (uint i = 0; i < idx_mat.size(); i++) {
            for (uint j = 0; j < idx_mat[i].size(); j++) {
                if (idx_mat[i][j] != -1) {
                    std::size_t idx = idx_mat[i][j];
                    added_pt.x = raw_cloud.points[idx].x;
                    added_pt.y = raw_cloud.points[idx].y;
                    added_pt.z = raw_cloud.points[idx].z;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = raw_cloud.points[idx].timestamp /* -min_time */;
                    added_pt.label = 0;
                    cloud->push_back(added_pt);
                } else {
                    added_pt.x = 10000.0;
                    added_pt.y = 10000.0;
                    added_pt.z = 10000.0;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = -10;
                    added_pt.label = 1000; // 无效点
                    cloud->push_back(added_pt);
                }
            }
        }
    }

    Lidar_Format_Processor::Data lidar_data;
    lidar_data.cloud = cloud;
    lidar_data.begin_time = min_time;
    lidar_data.end_time = max_time;
    lidar_data.valid_pt_num = valid_size;
    return lidar_data;
}

Lidar_Format_Processor::Data Lidar_Format_Processor::labs_livox_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    pcl::PointCloud<PointXYZIRCT> raw_cloud;
    pcl::fromPCLPointCloud2(*msg, raw_cloud);
    double min_time = DBL_MAX, max_time = DBL_MIN;
    for (uint i = 0; i < raw_cloud.size(); i++) {
        // std::cout << raw_cloud.points[i].timestamp << std::endl;
        if (min_time > raw_cloud.points[i].timestamp) {
            min_time = raw_cloud.points[i].timestamp;
        }
        if (max_time < raw_cloud.points[i].timestamp) {
            max_time = raw_cloud.points[i].timestamp;
        }
    }
    if (max_time - min_time > 0.11) {
        max_time = min_time + 0.1;
    }

    pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    *cloud = raw_cloud;

    Lidar_Format_Processor::Data lidar_data;
    lidar_data.cloud = cloud;
    lidar_data.begin_time = min_time;
    lidar_data.end_time = lidar_data.begin_time + (max_time - min_time);
    lidar_data.valid_pt_num = cloud->size();
    return lidar_data;
}

Lidar_Format_Processor::Data Lidar_Format_Processor::hilti_hesai_PandarXT32_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    pcl::PointCloud<Hesai::PointXYZITR> raw_cloud;
    pcl::fromPCLPointCloud2(*msg, raw_cloud);
    double min_time = DBL_MAX, max_time = DBL_MIN;
    for (uint i = 0; i < raw_cloud.size(); i++) {
        if (min_time > raw_cloud.points[i].timestamp) {
            min_time = raw_cloud.points[i].timestamp;
        }
        if (max_time < raw_cloud.points[i].timestamp) {
            max_time = raw_cloud.points[i].timestamp;
        }
    }
    // std::cout << "min_time" << min_time << std::endl;
    // std::cout << "max_time" << max_time << std::endl;

    // pcl::PointCloud<Hesai::PointXYZITR> raw_cloud;
    // pcl::fromPCLPointCloud2(*msg, raw_cloud);
    // std::cout << "hilti_hesai_PandarXT32_handler 1" << std::endl;
    // int plsize = raw_cloud.size();
    // double min_time = DBL_MAX, max_time = DBL_MIN;
    // for(uint i = 0; i < plsize; i++) {
    // 	if(min_time > raw_cloud.points[i].timestamp) {
    // 		min_time = raw_cloud.points[i].timestamp;
    // 	}
    // 	if(max_time < raw_cloud.points[i].timestamp) {
    // 		max_time = raw_cloud.points[i].timestamp;
    // 	}
    // }

    // std::cout << "min_time" << min_time;
    // std::cout << "max_time" << max_time;
    pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    PointXYZIRCT added_pt;
    for (auto pt : raw_cloud) {
        added_pt.x = pt.x;
        added_pt.y = pt.y;
        added_pt.z = pt.z;
        added_pt.column = 0;
        added_pt.row = pt.ring;
        added_pt.timestamp = pt.timestamp;
        // added_pt.label = /* 0 */label_string[i*idx_mat[i].size()+j] - '0';
        cloud->push_back(added_pt);
    }

    // std::vector<int> column_statics(plsize);
    // for(int i = 0; i < plsize; i++) {
    // 	int ori = (M_PI - atan2(point.y, point.x))/param_lidar_image_col_resolution;
    // }

    // std::vector<std::vector<int>> idx_mat(param_lidar_image_rows_ , std::vector<int>(param_lidar_image_cols_, -1));
    // double time_interval = max_time - min_time;
    // double tmp = param_lidar_image_cols_ / time_interval;
    // int col_table_idx;
    // for(uint i = 0; i < plsize; i++) {
    // 	col_table_idx = (raw_cloud.points[i].time - min_time) * tmp;
    // 	idx_mat[raw_cloud.points[i].ring][col_table_idx] = i;
    // }

    // pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    // if(label_string.length() != 0) {
    // 	PointXYZIRCT added_pt;
    // 	for(uint i = 0; i < param_lidar_image_rows_; i++) {
    // 		for(uint j = 0; j < param_lidar_image_cols_; j++){
    // 			if(idx_mat[i][j] != -1) {
    // 				std::size_t idx = idx_mat[i][j];
    // 				added_pt.x = raw_cloud.points[idx].x;
    // 				added_pt.y = raw_cloud.points[idx].y;
    // 				added_pt.z = raw_cloud.points[idx].z;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = raw_cloud.points[idx].time;
    // 				added_pt.label = /* 0 */label_string[i*idx_mat[i].size()+j] - '0';
    // 				cloud->push_back(added_pt);
    // 			} else {
    // 				added_pt.x = 10000.0;
    // 				added_pt.y = 10000.0;
    // 				added_pt.z = 10000.0;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = -10;
    // 				added_pt.label = 1000;	// 无效点
    // 				cloud->push_back(added_pt);
    // 			}
    // 		}
    // 	}
    // } else {
    // 	PointXYZIRCT added_pt;
    // 	for(uint i = 0; i < param_lidar_image_rows_; i++) {
    // 		for(uint j = 0; j < param_lidar_image_cols_; j++){
    // 			if(idx_mat[i][j] != -1) {
    // 				std::size_t idx = idx_mat[i][j];
    // 				added_pt.x = raw_cloud.points[idx].x;
    // 				added_pt.y = raw_cloud.points[idx].y;
    // 				added_pt.z = raw_cloud.points[idx].z;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = raw_cloud.points[idx].time;
    // 				added_pt.label = 0;
    // 				cloud->push_back(added_pt);
    // 			} else {
    // 				added_pt.x = 10000.0;
    // 				added_pt.y = 10000.0;
    // 				added_pt.z = 10000.0;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = -10;
    // 				added_pt.label = 1000;	// 无效点
    // 				cloud->push_back(added_pt);
    // 			}
    // 		}
    // 	}
    // }

    Lidar_Format_Processor::Data lidar_data;
    lidar_data.cloud = cloud;
    lidar_data.begin_time = min_time;
    lidar_data.end_time = lidar_data.begin_time + (max_time - min_time);
    lidar_data.valid_pt_num = /* plsize */ 0;
    std::cout << "hilti_hesai_PandarXT32_handler 2 " << std::endl;
    return lidar_data;
}

Lidar_Format_Processor::Data Lidar_Format_Processor::urbanloco_velodyne_HDL32E_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    pcl::PointCloud<Velodyne::PointXYZIR> raw_cloud;
    pcl::fromPCLPointCloud2(*msg, raw_cloud);

    int plsize = raw_cloud.size();
    double min_time = 0, max_time = 0.1;
    float startOri = -atan2(raw_cloud.points[0].y, raw_cloud.points[0].x);
    float endOri = -atan2(raw_cloud.points[plsize - 1].y, raw_cloud.points[plsize - 1].x) + 2 * M_PI;
    if ((endOri - startOri) > 3 * M_PI) {
        endOri -= 2 * M_PI;
    } else if ((endOri - startOri) < M_PI) {
        endOri += 2 * M_PI;
    }
    bool halfPassed = false;
    PointXYZIRCT point;
    float relTime;
    pcl::PointCloud<PointXYZIRCT>::Ptr cloud_tmp(new pcl::PointCloud<PointXYZIRCT>());
    for (int i = 0; i < plsize; i++) {
        point.x = raw_cloud.points[i].x;
        point.y = raw_cloud.points[i].y;
        point.z = raw_cloud.points[i].z;

        float ori = -atan2(point.y, point.x);
        if (!halfPassed) {
            if (ori < startOri - M_PI / 2)
                ori += 2 * M_PI;
            else if (ori > startOri + M_PI * 3 / 2)
                ori -= 2 * M_PI;

            if (ori - startOri > M_PI)
                halfPassed = true;
        } else {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
                ori += 2 * M_PI;
            else if (ori > endOri + M_PI / 2)
                ori -= 2 * M_PI;
        }

        float relTime = (ori - startOri) / (endOri - startOri);

        point.timestamp = 0.1 * relTime + timestamp;
        point.row = raw_cloud.points[i].ring;
        ;
        cloud_tmp->push_back(point);
    }

    Lidar_Format_Processor::Data lidar_data;
    lidar_data.cloud = cloud_tmp;
    lidar_data.begin_time = timestamp;
    lidar_data.end_time = lidar_data.begin_time + (max_time - min_time);
    lidar_data.valid_pt_num = plsize;
    return lidar_data;
}

Lidar_Format_Processor::Data Lidar_Format_Processor::nclt_velodyne_HDL32E_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    pcl::PointCloud<Velodyne::PointXYZRT> raw_cloud;
    pcl::fromPCLPointCloud2(*msg, raw_cloud);
    int plsize = raw_cloud.size();
    double min_time = DBL_MAX, max_time = DBL_MIN;
    for (uint i = 0; i < plsize; i++) {
        if (min_time > raw_cloud.points[i].time) {
            min_time = raw_cloud.points[i].time;
        }
        if (max_time < raw_cloud.points[i].time) {
            max_time = raw_cloud.points[i].time;
        }
    }

    // if(max_time * 0.000001 - min_time * 0.000001 > 0.11) {
    // 	max_time = min_time+100000;
    // }

    pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    PointXYZIRCT added_pt;
    for (auto pt : raw_cloud) {
        added_pt.x = pt.x;
        added_pt.y = pt.y;
        added_pt.z = pt.z;
        added_pt.column = 0;
        added_pt.row = pt.ring;
        added_pt.timestamp = double(pt.time * 0.000001) + timestamp;
        // added_pt.label = /* 0 */label_string[i*idx_mat[i].size()+j] - '0';
        cloud->push_back(added_pt);
        // std::cout << pt.time << " ";
    }

    Lidar_Format_Processor::Data lidar_data;
    lidar_data.cloud = cloud;
    lidar_data.begin_time = timestamp;
    lidar_data.end_time = lidar_data.begin_time + (max_time - min_time) * 0.000001;
    lidar_data.valid_pt_num = plsize;
    return lidar_data;
}

Lidar_Format_Processor::Data Lidar_Format_Processor::urbannav_new_velodyne_HDL32E_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    pcl::PointCloud<Velodyne::PointXYZIRT> raw_cloud;
    pcl::fromPCLPointCloud2(*msg, raw_cloud);
    int plsize = raw_cloud.size();
    double min_time = DBL_MAX, max_time = DBL_MIN;
    for (uint i = 0; i < plsize; i++) {
        if (min_time > raw_cloud.points[i].time) {
            min_time = raw_cloud.points[i].time;
        }
        if (max_time < raw_cloud.points[i].time) {
            max_time = raw_cloud.points[i].time;
        }
    }

    pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    PointXYZIRCT added_pt;
    for (auto pt : raw_cloud) {
        added_pt.x = pt.x;
        added_pt.y = pt.y;
        added_pt.z = pt.z;
        added_pt.column = 0;
        added_pt.row = pt.ring;
        added_pt.timestamp = double(pt.time) + timestamp;
        // added_pt.label = /* 0 */label_string[i*idx_mat[i].size()+j] - '0';
        cloud->push_back(added_pt);
        // std::cout << pt.time << " ";
    }

    // std::vector<int> column_statics(plsize);
    // for(int i = 0; i < plsize; i++) {
    // 	int ori = (M_PI - atan2(point.y, point.x))/param_lidar_image_col_resolution;
    // }

    // std::vector<std::vector<int>> idx_mat(param_lidar_image_rows_ , std::vector<int>(param_lidar_image_cols_, -1));
    // double time_interval = max_time - min_time;
    // double tmp = param_lidar_image_cols_ / time_interval;
    // int col_table_idx;
    // for(uint i = 0; i < plsize; i++) {
    // 	col_table_idx = (raw_cloud.points[i].time - min_time) * tmp;
    // 	idx_mat[raw_cloud.points[i].ring][col_table_idx] = i;
    // }

    // pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    // if(label_string.length() != 0) {
    // 	PointXYZIRCT added_pt;
    // 	for(uint i = 0; i < param_lidar_image_rows_; i++) {
    // 		for(uint j = 0; j < param_lidar_image_cols_; j++){
    // 			if(idx_mat[i][j] != -1) {
    // 				std::size_t idx = idx_mat[i][j];
    // 				added_pt.x = raw_cloud.points[idx].x;
    // 				added_pt.y = raw_cloud.points[idx].y;
    // 				added_pt.z = raw_cloud.points[idx].z;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = raw_cloud.points[idx].time;
    // 				added_pt.label = /* 0 */label_string[i*idx_mat[i].size()+j] - '0';
    // 				cloud->push_back(added_pt);
    // 			} else {
    // 				added_pt.x = 10000.0;
    // 				added_pt.y = 10000.0;
    // 				added_pt.z = 10000.0;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = -10;
    // 				added_pt.label = 1000;	// 无效点
    // 				cloud->push_back(added_pt);
    // 			}
    // 		}
    // 	}
    // } else {
    // 	PointXYZIRCT added_pt;
    // 	for(uint i = 0; i < param_lidar_image_rows_; i++) {
    // 		for(uint j = 0; j < param_lidar_image_cols_; j++){
    // 			if(idx_mat[i][j] != -1) {
    // 				std::size_t idx = idx_mat[i][j];
    // 				added_pt.x = raw_cloud.points[idx].x;
    // 				added_pt.y = raw_cloud.points[idx].y;
    // 				added_pt.z = raw_cloud.points[idx].z;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = raw_cloud.points[idx].time;
    // 				added_pt.label = 0;
    // 				cloud->push_back(added_pt);
    // 			} else {
    // 				added_pt.x = 10000.0;
    // 				added_pt.y = 10000.0;
    // 				added_pt.z = 10000.0;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = -10;
    // 				added_pt.label = 1000;	// 无效点
    // 				cloud->push_back(added_pt);
    // 			}
    // 		}
    // 	}
    // }

    Lidar_Format_Processor::Data lidar_data;
    lidar_data.cloud = cloud;
    if (max_time > 0.02 && min_time > -0.02) {
        lidar_data.begin_time = timestamp;
        lidar_data.end_time = lidar_data.begin_time + (max_time - min_time);
    } else {
        lidar_data.begin_time = timestamp - (max_time - min_time);
        lidar_data.end_time = timestamp;
    }
    lidar_data.valid_pt_num = plsize;
    return lidar_data;
}

Lidar_Format_Processor::Data Lidar_Format_Processor::ntuviral_ouster_OS1_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    pcl::PointCloud<Ouster::PointXYZITRRNR> raw_cloud;
    pcl::fromPCLPointCloud2(*msg, raw_cloud);
    uint32_t min_time = UINT32_MAX, max_time = 0;
    for (uint i = 0; i < raw_cloud.size(); i++) {
        if (min_time > raw_cloud.points[i].t) {
            min_time = raw_cloud.points[i].t;
        }
        if (max_time < raw_cloud.points[i].t) {
            max_time = raw_cloud.points[i].t;
        }
    }
    // std::cout << "min_time" << min_time << std::endl;
    // std::cout << "max_time" << max_time << std::endl;

    // pcl::PointCloud<Hesai::PointXYZITR> raw_cloud;
    // pcl::fromPCLPointCloud2(*msg, raw_cloud);
    // std::cout << "hilti_hesai_PandarXT32_handler 1" << std::endl;
    // int plsize = raw_cloud.size();
    // double min_time = DBL_MAX, max_time = DBL_MIN;
    // for(uint i = 0; i < plsize; i++) {
    // 	if(min_time > raw_cloud.points[i].timestamp) {
    // 		min_time = raw_cloud.points[i].timestamp;
    // 	}
    // 	if(max_time < raw_cloud.points[i].timestamp) {
    // 		max_time = raw_cloud.points[i].timestamp;
    // 	}
    // }

    // std::cout << "min_time" << min_time;
    // std::cout << "max_time" << max_time;
    pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    PointXYZIRCT added_pt;
    for (auto pt : raw_cloud) {
        added_pt.x = pt.x;
        added_pt.y = pt.y;
        added_pt.z = pt.z;
        added_pt.column = 0;
        added_pt.row = pt.ring;
        added_pt.timestamp = pt.t / 1000000000.0 + timestamp;
        // added_pt.label = /* 0 */label_string[i*idx_mat[i].size()+j] - '0';
        cloud->push_back(added_pt);
    }

    // std::vector<int> column_statics(plsize);
    // for(int i = 0; i < plsize; i++) {
    // 	int ori = (M_PI - atan2(point.y, point.x))/param_lidar_image_col_resolution;
    // }

    // std::vector<std::vector<int>> idx_mat(param_lidar_image_rows_ , std::vector<int>(param_lidar_image_cols_, -1));
    // double time_interval = max_time - min_time;
    // double tmp = param_lidar_image_cols_ / time_interval;
    // int col_table_idx;
    // for(uint i = 0; i < plsize; i++) {
    // 	col_table_idx = (raw_cloud.points[i].time - min_time) * tmp;
    // 	idx_mat[raw_cloud.points[i].ring][col_table_idx] = i;
    // }

    // pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    // if(label_string.length() != 0) {
    // 	PointXYZIRCT added_pt;
    // 	for(uint i = 0; i < param_lidar_image_rows_; i++) {
    // 		for(uint j = 0; j < param_lidar_image_cols_; j++){
    // 			if(idx_mat[i][j] != -1) {
    // 				std::size_t idx = idx_mat[i][j];
    // 				added_pt.x = raw_cloud.points[idx].x;
    // 				added_pt.y = raw_cloud.points[idx].y;
    // 				added_pt.z = raw_cloud.points[idx].z;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = raw_cloud.points[idx].time;
    // 				added_pt.label = /* 0 */label_string[i*idx_mat[i].size()+j] - '0';
    // 				cloud->push_back(added_pt);
    // 			} else {
    // 				added_pt.x = 10000.0;
    // 				added_pt.y = 10000.0;
    // 				added_pt.z = 10000.0;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = -10;
    // 				added_pt.label = 1000;	// 无效点
    // 				cloud->push_back(added_pt);
    // 			}
    // 		}
    // 	}
    // } else {
    // 	PointXYZIRCT added_pt;
    // 	for(uint i = 0; i < param_lidar_image_rows_; i++) {
    // 		for(uint j = 0; j < param_lidar_image_cols_; j++){
    // 			if(idx_mat[i][j] != -1) {
    // 				std::size_t idx = idx_mat[i][j];
    // 				added_pt.x = raw_cloud.points[idx].x;
    // 				added_pt.y = raw_cloud.points[idx].y;
    // 				added_pt.z = raw_cloud.points[idx].z;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = raw_cloud.points[idx].time;
    // 				added_pt.label = 0;
    // 				cloud->push_back(added_pt);
    // 			} else {
    // 				added_pt.x = 10000.0;
    // 				added_pt.y = 10000.0;
    // 				added_pt.z = 10000.0;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = -10;
    // 				added_pt.label = 1000;	// 无效点
    // 				cloud->push_back(added_pt);
    // 			}
    // 		}
    // 	}
    // }

    Lidar_Format_Processor::Data lidar_data;
    lidar_data.cloud = cloud;
    lidar_data.begin_time = timestamp + min_time / 1000000000.0;
    lidar_data.end_time = timestamp + max_time / 1000000000.0;
    lidar_data.valid_pt_num = /* plsize */ 0;
    // std::cout << "hilti_hesai_PandarXT32_handler 2 " << std::endl;
    return lidar_data;
}

Lidar_Format_Processor::Data Lidar_Format_Processor::newercollege_ouster_OS1_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    pcl::PointCloud<Ouster::PointXYZITRRNR> raw_cloud;
    pcl::fromPCLPointCloud2(*msg, raw_cloud);
    uint32_t min_time = UINT32_MAX, max_time = 0;
    for (uint i = 0; i < raw_cloud.size(); i++) {
        if (min_time > raw_cloud.points[i].t) {
            min_time = raw_cloud.points[i].t;
        }
        if (max_time < raw_cloud.points[i].t) {
            max_time = raw_cloud.points[i].t;
        }
    }
    // if(max_time/1000000000.0 - min_time/1000000000.0 > 0.11) {
    // 	max_time = min_time+0.1*1000000000.0;
    // }
    // std::cout << "min_time" << min_time << std::endl;
    // std::cout << "max_time" << max_time << std::endl;

    // pcl::PointCloud<Hesai::PointXYZITR> raw_cloud;
    // pcl::fromPCLPointCloud2(*msg, raw_cloud);
    // std::cout << "hilti_hesai_PandarXT32_handler 1" << std::endl;
    // int plsize = raw_cloud.size();
    // double min_time = DBL_MAX, max_time = DBL_MIN;
    // for(uint i = 0; i < plsize; i++) {
    // 	if(min_time > raw_cloud.points[i].timestamp) {
    // 		min_time = raw_cloud.points[i].timestamp;
    // 	}
    // 	if(max_time < raw_cloud.points[i].timestamp) {
    // 		max_time = raw_cloud.points[i].timestamp;
    // 	}
    // }

    // std::cout << "min_time" << min_time;
    // std::cout << "max_time" << max_time;
    pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    PointXYZIRCT added_pt;
    for (auto pt : raw_cloud) {
        added_pt.x = pt.x;
        added_pt.y = pt.y;
        added_pt.z = pt.z;
        added_pt.column = 0;
        added_pt.row = pt.ring;
        added_pt.timestamp = pt.t / 1000000000.0 + timestamp;
        // added_pt.label = /* 0 */label_string[i*idx_mat[i].size()+j] - '0';
        cloud->push_back(added_pt);
    }

    // std::vector<int> column_statics(plsize);
    // for(int i = 0; i < plsize; i++) {
    // 	int ori = (M_PI - atan2(point.y, point.x))/param_lidar_image_col_resolution;
    // }

    // std::vector<std::vector<int>> idx_mat(param_lidar_image_rows_ , std::vector<int>(param_lidar_image_cols_, -1));
    // double time_interval = max_time - min_time;
    // double tmp = param_lidar_image_cols_ / time_interval;
    // int col_table_idx;
    // for(uint i = 0; i < plsize; i++) {
    // 	col_table_idx = (raw_cloud.points[i].time - min_time) * tmp;
    // 	idx_mat[raw_cloud.points[i].ring][col_table_idx] = i;
    // }

    // pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    // if(label_string.length() != 0) {
    // 	PointXYZIRCT added_pt;
    // 	for(uint i = 0; i < param_lidar_image_rows_; i++) {
    // 		for(uint j = 0; j < param_lidar_image_cols_; j++){
    // 			if(idx_mat[i][j] != -1) {
    // 				std::size_t idx = idx_mat[i][j];
    // 				added_pt.x = raw_cloud.points[idx].x;
    // 				added_pt.y = raw_cloud.points[idx].y;
    // 				added_pt.z = raw_cloud.points[idx].z;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = raw_cloud.points[idx].time;
    // 				added_pt.label = /* 0 */label_string[i*idx_mat[i].size()+j] - '0';
    // 				cloud->push_back(added_pt);
    // 			} else {
    // 				added_pt.x = 10000.0;
    // 				added_pt.y = 10000.0;
    // 				added_pt.z = 10000.0;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = -10;
    // 				added_pt.label = 1000;	// 无效点
    // 				cloud->push_back(added_pt);
    // 			}
    // 		}
    // 	}
    // } else {
    // 	PointXYZIRCT added_pt;
    // 	for(uint i = 0; i < param_lidar_image_rows_; i++) {
    // 		for(uint j = 0; j < param_lidar_image_cols_; j++){
    // 			if(idx_mat[i][j] != -1) {
    // 				std::size_t idx = idx_mat[i][j];
    // 				added_pt.x = raw_cloud.points[idx].x;
    // 				added_pt.y = raw_cloud.points[idx].y;
    // 				added_pt.z = raw_cloud.points[idx].z;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = raw_cloud.points[idx].time;
    // 				added_pt.label = 0;
    // 				cloud->push_back(added_pt);
    // 			} else {
    // 				added_pt.x = 10000.0;
    // 				added_pt.y = 10000.0;
    // 				added_pt.z = 10000.0;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = -10;
    // 				added_pt.label = 1000;	// 无效点
    // 				cloud->push_back(added_pt);
    // 			}
    // 		}
    // 	}
    // }

    Lidar_Format_Processor::Data lidar_data;
    lidar_data.cloud = cloud;
    lidar_data.begin_time = timestamp;
    lidar_data.end_time = timestamp + max_time / 1000000000.0;
    if (lidar_data.end_time - lidar_data.begin_time > 0.11) {
        lidar_data.end_time = lidar_data.begin_time + 0.1;
    }
    lidar_data.valid_pt_num = /* plsize */ 0;
    // std::cout << "hilti_hesai_PandarXT32_handler 2 " << std::endl;
    return lidar_data;
}

Lidar_Format_Processor::Data Lidar_Format_Processor::labs_robosense_RSLiDAR16_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    pcl::PointCloud<RoboSense::PointXYZIRT> raw_cloud;
    pcl::fromPCLPointCloud2(*msg, raw_cloud);

    int plsize = raw_cloud.size();
    double min_time = DBL_MAX, max_time = DBL_MIN;
    for (uint i = 0; i < plsize; i++) {
        if (min_time > raw_cloud.points[i].timestamp) {
            min_time = raw_cloud.points[i].timestamp;
        }
        if (max_time < raw_cloud.points[i].timestamp) {
            max_time = raw_cloud.points[i].timestamp;
        }
    }

    std::vector<std::vector<int>> idx_mat(param_lidar_image_rows_, std::vector<int>(param_lidar_image_cols_, -1));
    double time_interval = max_time - min_time;
    double tmp = param_lidar_image_cols_ / time_interval;
    int col_table_idx;
    for (uint i = 0; i < plsize; i++) {
        col_table_idx = (raw_cloud.points[i].timestamp - min_time) * tmp;
        idx_mat[raw_cloud.points[i].ring][col_table_idx] = i;
    }

    pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    if (label_string.length() != 0) {
        PointXYZIRCT added_pt;
        for (uint i = 0; i < param_lidar_image_rows_; i++) {
            for (uint j = 0; j < param_lidar_image_cols_; j++) {
                if (idx_mat[i][j] != -1) {
                    std::size_t idx = idx_mat[i][j];
                    added_pt.x = raw_cloud.points[idx].x;
                    added_pt.y = raw_cloud.points[idx].y;
                    added_pt.z = raw_cloud.points[idx].z;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = raw_cloud.points[idx].timestamp - min_time;
                    added_pt.label = /* 0 */ label_string[i * idx_mat[i].size() + j] - '0';
                    cloud->push_back(added_pt);
                } else {
                    added_pt.x = 10000.0;
                    added_pt.y = 10000.0;
                    added_pt.z = 10000.0;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = -10;
                    added_pt.label = 1000; // 无效点
                    cloud->push_back(added_pt);
                }
            }
        }
    } else {
        PointXYZIRCT added_pt;
        for (uint i = 0; i < param_lidar_image_rows_; i++) {
            for (uint j = 0; j < param_lidar_image_cols_; j++) {
                if (idx_mat[i][j] != -1) {
                    std::size_t idx = idx_mat[i][j];
                    added_pt.x = raw_cloud.points[idx].x;
                    added_pt.y = raw_cloud.points[idx].y;
                    added_pt.z = raw_cloud.points[idx].z;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = raw_cloud.points[idx].timestamp - min_time;
                    added_pt.label = 0;
                    cloud->push_back(added_pt);
                } else {
                    added_pt.x = 10000.0;
                    added_pt.y = 10000.0;
                    added_pt.z = 10000.0;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = -10;
                    added_pt.label = 1000; // 无效点
                    cloud->push_back(added_pt);
                }
            }
        }
    }

    Lidar_Format_Processor::Data lidar_data;
    lidar_data.cloud = cloud;
    lidar_data.begin_time = min_time;
    lidar_data.end_time = max_time;
    lidar_data.valid_pt_num = plsize;
    return lidar_data;
}

Lidar_Format_Processor::Data Lidar_Format_Processor::labs_velodyne_VLP16_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    pcl::PointCloud<Velodyne::PointXYZIRT> raw_cloud;
    pcl::fromPCLPointCloud2(*msg, raw_cloud);

    int plsize = raw_cloud.size();
    double min_time = DBL_MAX, max_time = DBL_MIN;
    for (uint i = 0; i < plsize; i++) {
        if (min_time > raw_cloud.points[i].time) {
            min_time = raw_cloud.points[i].time;
        }
        if (max_time < raw_cloud.points[i].time) {
            max_time = raw_cloud.points[i].time;
        }
    }

    std::vector<std::vector<int>> idx_mat(param_lidar_image_rows_, std::vector<int>(param_lidar_image_cols_, -1));
    double time_interval = max_time - min_time;
    double tmp = param_lidar_image_cols_ / time_interval;
    int col_table_idx;
    for (uint i = 0; i < plsize; i++) {
        col_table_idx = (raw_cloud.points[i].time - min_time) * tmp;
        idx_mat[raw_cloud.points[i].ring][col_table_idx] = i;
    }

    pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    if (label_string.length() != 0) {
        PointXYZIRCT added_pt;
        for (uint i = 0; i < param_lidar_image_rows_; i++) {
            for (uint j = 0; j < param_lidar_image_cols_; j++) {
                if (idx_mat[i][j] != -1) {
                    std::size_t idx = idx_mat[i][j];
                    added_pt.x = raw_cloud.points[idx].x;
                    added_pt.y = raw_cloud.points[idx].y;
                    added_pt.z = raw_cloud.points[idx].z;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = raw_cloud.points[idx].time;
                    added_pt.label = /* 0 */ label_string[i * idx_mat[i].size() + j] - '0';
                    cloud->push_back(added_pt);
                } else {
                    added_pt.x = 10000.0;
                    added_pt.y = 10000.0;
                    added_pt.z = 10000.0;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = -10;
                    added_pt.label = 1000; // 无效点
                    cloud->push_back(added_pt);
                }
            }
        }
    } else {
        PointXYZIRCT added_pt;
        for (uint i = 0; i < param_lidar_image_rows_; i++) {
            for (uint j = 0; j < param_lidar_image_cols_; j++) {
                if (idx_mat[i][j] != -1) {
                    std::size_t idx = idx_mat[i][j];
                    added_pt.x = raw_cloud.points[idx].x;
                    added_pt.y = raw_cloud.points[idx].y;
                    added_pt.z = raw_cloud.points[idx].z;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = raw_cloud.points[idx].time;
                    added_pt.label = 0;
                    cloud->push_back(added_pt);
                } else {
                    added_pt.x = 10000.0;
                    added_pt.y = 10000.0;
                    added_pt.z = 10000.0;
                    added_pt.column = j;
                    added_pt.row = i;
                    added_pt.timestamp = -10;
                    added_pt.label = 1000; // 无效点
                    cloud->push_back(added_pt);
                }
            }
        }
    }

    Lidar_Format_Processor::Data lidar_data;
    lidar_data.cloud = cloud;
    lidar_data.begin_time = timestamp;
    lidar_data.end_time = lidar_data.begin_time + (max_time - min_time);
    lidar_data.valid_pt_num = plsize;
    return lidar_data;
}

Lidar_Format_Processor::Data Lidar_Format_Processor::urbannav_velodyne_HDL32E_handler(double timestamp, const pcl::PCLPointCloud2::Ptr &msg, const std::string &label_string)
{
    pcl::PointCloud<Velodyne::PointXYZIR> raw_cloud;
    pcl::fromPCLPointCloud2(*msg, raw_cloud);

    int plsize = raw_cloud.size();
    double min_time = 0, max_time = 0.1;
    float startOri = -atan2(raw_cloud.points[0].y, raw_cloud.points[0].x);
    float endOri = -atan2(raw_cloud.points[plsize - 1].y, raw_cloud.points[plsize - 1].x) + 2 * M_PI;
    if ((endOri - startOri) > 3 * M_PI) {
        endOri -= 2 * M_PI;
    } else if ((endOri - startOri) < M_PI) {
        endOri += 2 * M_PI;
    }
    bool halfPassed = false;
    PointXYZIRCT point;
    float relTime;
    pcl::PointCloud<PointXYZIRCT>::Ptr cloud_tmp(new pcl::PointCloud<PointXYZIRCT>());
    for (int i = 0; i < plsize; i++) {
        point.x = raw_cloud.points[i].x;
        point.y = raw_cloud.points[i].y;
        point.z = raw_cloud.points[i].z;

        float ori = -atan2(point.y, point.x);
        if (!halfPassed) {
            if (ori < startOri - M_PI / 2)
                ori += 2 * M_PI;
            else if (ori > startOri + M_PI * 3 / 2)
                ori -= 2 * M_PI;

            if (ori - startOri > M_PI)
                halfPassed = true;
        } else {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
                ori += 2 * M_PI;
            else if (ori > endOri + M_PI / 2)
                ori -= 2 * M_PI;
        }

        float relTime = (ori - startOri) / (endOri - startOri);

        point.timestamp = 0.1 * relTime + timestamp;
        point.row = raw_cloud.points[i].ring;
        ;
        cloud_tmp->push_back(point);
    }

    // std::vector<std::vector<int>> idx_mat(param_lidar_image_rows_ , std::vector<int>(param_lidar_image_cols_, -1));
    // double time_interval = max_time - min_time;
    // double tmp = param_lidar_image_cols_ / time_interval;
    // int col_table_idx;
    // for(uint i = 0; i < plsize; i++) {
    // 	col_table_idx = (cloud_tmp.points[i].timestamp - min_time) * tmp;
    // 	idx_mat[cloud_tmp.points[i].row][col_table_idx] = i;
    // }

    // pcl::PointCloud<PointXYZIRCT>::Ptr cloud(new pcl::PointCloud<PointXYZIRCT>());
    // if(label_string.length() != 0) {
    // 	PointXYZIRCT added_pt;
    // 	for(uint i = 0; i < param_lidar_image_rows_; i++) {
    // 		for(uint j = 0; j < param_lidar_image_cols_; j++){
    // 			if(idx_mat[i][j] != -1) {
    // 				std::size_t idx = idx_mat[i][j];
    // 				added_pt.x = cloud_tmp.points[idx].x;
    // 				added_pt.y = cloud_tmp.points[idx].y;
    // 				added_pt.z = cloud_tmp.points[idx].z;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = cloud_tmp.points[idx].timestamp;
    // 				added_pt.label = /* 0 */label_string[i*idx_mat[i].size()+j] - '0';
    // 				cloud->push_back(added_pt);
    // 			} else {
    // 				added_pt.x = 10000.0;
    // 				added_pt.y = 10000.0;
    // 				added_pt.z = 10000.0;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = -10;
    // 				added_pt.label = 1000;	// 无效点
    // 				cloud->push_back(added_pt);
    // 			}
    // 		}
    // 	}
    // } else {
    // 	PointXYZIRCT added_pt;
    // 	for(uint i = 0; i < param_lidar_image_rows_; i++) {
    // 		for(uint j = 0; j < param_lidar_image_cols_; j++){
    // 			if(idx_mat[i][j] != -1) {
    // 				std::size_t idx = idx_mat[i][j];
    // 				added_pt.x = cloud_tmp.points[idx].x;
    // 				added_pt.y = cloud_tmp.points[idx].y;
    // 				added_pt.z = cloud_tmp.points[idx].z;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = cloud_tmp.points[idx].timestamp;
    // 				added_pt.label = 0;
    // 				cloud->push_back(added_pt);
    // 			} else {
    // 				added_pt.x = 10000.0;
    // 				added_pt.y = 10000.0;
    // 				added_pt.z = 10000.0;
    // 				added_pt.column = j;
    // 				added_pt.row = i;
    // 				added_pt.timestamp = -10;
    // 				added_pt.label = 1000;	// 无效点
    // 				cloud->push_back(added_pt);
    // 			}
    // 		}
    // 	}
    // }

    Lidar_Format_Processor::Data lidar_data;
    lidar_data.cloud = cloud_tmp;
    lidar_data.begin_time = timestamp;
    lidar_data.end_time = lidar_data.begin_time + (max_time - min_time);
    lidar_data.valid_pt_num = plsize;
    return lidar_data;
}