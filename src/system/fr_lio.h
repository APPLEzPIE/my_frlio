#pragma once
#include <omp.h>
#include <thread>
#include <mutex>
#include <csignal>
#include <unistd.h>
#include <algorithm>
//#include <Python.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "../basic_element/extrinsic_group.hpp"
#include "../basic_element/camera_intrinsic.hpp"
#include "../pointcloud_map/rc_vox_map.hpp"
#include "./subframe_lio.h"
#include "./lidar_motion_compensate_tool.h"
#include "../sensor/sensor_synchronizer.h"
#include "../tools/tools.hpp"
#include "./associated_pair.hpp"
#include "../vision/rgb_keyframe_generation.hpp"
#include "../../src/tools/time_tools.hpp"

/**
 * @brief FR-LIO主里程计系统。
 *
 * 主流程概览：
 * 1. 读取系统/传感器配置并初始化各子模块。
 * 2. 通过 Sensor_Synchronizer 获取 LiDAR-IMU（可选图像）同步测量。
 * 3. 完成运动补偿、子帧划分、地图关联与迭代优化。
 * 4. 在退化场景下结合平滑与状态约束保证稳定输出。
 * 5. 可选生成RGB关键帧子地图用于可视化/离线建图。
 */
template <typename PointType> class Odometry {
public:
#ifdef MY_DEBUG_MODE
    struct Frame_Info {
        Frame_Info()
            : pt_num_(0)
            , associated_num_(0)
            , subframe_num_(0)
            , opt_iteration_num_(0)
            , process_time_(0)
            , flg_valid_(false)
        {
        }

        Frame_Info(const State_Group &state)
            : state_(state)
        {
        }

        Frame_Info(const State_Group &state, int subframe_num)
            : flg_valid_(true)
            , state_(state)
            , subframe_num_(subframe_num)
        {
        }

        Frame_Info(const State_Group &state, int subframe_num, int pt_num, int associated_num)
            : flg_valid_(true)
            , state_(state)
            , subframe_num_(subframe_num)
            , pt_num_(pt_num)
            , associated_num_(associated_num)
        {
        }

        State_Group state_;
        int pt_num_;
        int associated_num_;
        int subframe_num_;
        int opt_iteration_num_;
        double process_time_;
        bool flg_valid_;
    };
#endif

    Odometry(const std::string &system_dir, const std::string &sensor_config_file)
        : system_dir_(system_dir)
        , sensor_config_file_(sensor_config_file)
        , flg_need_init_(true)
        , frame_id_count_(0)
        , associated_pairs_(20000)
#ifdef GEN_RGB_MAP
        , rbg_keyframe_gen_(&cam_intrinsic_)
#endif
    {
        if (system_dir_.back() != '/') {
            system_dir_ += "/";
        }

#if MY_VIS_MODE
        map_cloud_tmp_.reset(new typename pcl::PointCloud<PointType>());
        map_cloud_tmp_->reserve(500000);
#endif

#if MY_DEBUG_MODE
        predict_undistorted_cloud_.reset(new typename pcl::PointCloud<PointType>());
        predict_undistorted_cloud_->reserve(500000);

        predict_distorted_cloud_.reset(new typename pcl::PointCloud<PointType>());
        predict_distorted_cloud_->reserve(500000);
#endif
    }

    void load_config_parameter()
    {
        // 1) 读取系统级参数（优化配置、退化阈值、IMU噪声策略等）。
        std::string config_file = system_dir_ + "system/fr_lio.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file);

        try {
            param_flg_auto_subframe_num_analysis_ = config_node["param_flg_auto_subframe_num_analysis"].as<bool>();
            param_flg_auto_imu_noise_analysis_ = config_node["param_flg_auto_imu_noise_analysis"].as<bool>();
            param_min_subframe_num_ = config_node["param_min_subframe_num"].as<int>();
            param_max_subframe_num_ = config_node["param_max_subframe_num"].as<int>();
            param_max_imu_acc_noise_inc_ = config_node["param_max_imu_acc_noise_inc"].as<double>();
            param_max_imu_gyr_noise_inc_ = config_node["param_max_imu_gyr_noise_inc"].as<double>();
            param_flg_use_smooth_ = config_node["param_flg_use_smooth"].as<bool>();
            param_smooth_iteration_num_ = config_node["param_smooth_iteration_num"].as<int>();
            param_thres_degrade_pos_ = config_node["param_thres_degrade_pos"].as<double>();
            param_thres_degrade_ang_ = config_node["param_thres_degrade_ang"].as<double>() * M_PI / 180.0;

            param_min_point_num_for_registration_ = config_node["param_min_point_num_for_registration"].as<int>();
            param_min_point_num_for_downsample_ = config_node["param_min_point_num_for_downsample"].as<int>();
            param_max_iteration_num_max_ = config_node["param_max_iteration_num_max"].as<int>();
            param_max_rematch_num_max_ = config_node["param_max_rematch_num_max"].as<int>();
            param_max_iteration_num_min_ = config_node["param_max_iteration_num_min"].as<int>();
            param_max_rematch_num_min_ = config_node["param_max_rematch_num_min"].as<int>();
            param_max_pt_searched_num_ = config_node["param_max_pt_searched_num"].as<int>();
            param_min_pt_searched_num_ = config_node["param_min_pt_searched_num"].as<int>();
            param_angle_converage_thres_ = config_node["param_angle_converage_thres"].as<double>();
            param_position_converage_thres_ = config_node["param_position_converage_thres"].as<double>();

            param_gravity_value_ = config_node["param_gravity_value"].as<double>();
            param_point_filter_num_ = config_node["param_point_filter_num"].as<int>();
            param_lidar_points_downsample_resolution_ = config_node["param_lidar_points_downsample_resolution"].as<double>();
            param_flg_downsample_lidar_points_before_split_ = config_node["param_flg_downsample_lidar_points_before_split"].as<bool>();

            param_initialization_method_ = config_node["param_initialization_method"].as<int>();
            param_static_initialize_time_ = config_node["param_static_initialize_time"].as<double>();
            param_flg_estimate_imu_noise_ = config_node["param_flg_estimate_imu_noise"].as<bool>();

            param_pos_error_thres_ = config_node["param_pos_error_thres"].as<double>();
            param_ang_error_thres_ = config_node["param_ang_error_thres"].as<double>();
        } catch (...) {
            LOG(FATAL) << "[Odometry]: file " << config_file << " has a bad conversion";
        }

        // 第一阶段鲁棒性增强参数：缺省值内置，配置文件可按需覆盖。
        if (config_node["param_high_dynamic_ratio_thres"]) {
            param_high_dynamic_ratio_thres_ = config_node["param_high_dynamic_ratio_thres"].as<double>();
        }
        if (config_node["param_high_dynamic_q_scale"]) {
            param_high_dynamic_q_scale_ = config_node["param_high_dynamic_q_scale"].as<double>();
        }
        if (config_node["param_flg_residual_gating"]) {
            param_flg_residual_gating_ = config_node["param_flg_residual_gating"].as<bool>();
        }
        if (config_node["param_residual_gate_base"]) {
            param_residual_gate_base_ = config_node["param_residual_gate_base"].as<double>();
        }
        if (config_node["param_residual_gate_range_scale"]) {
            param_residual_gate_range_scale_ = config_node["param_residual_gate_range_scale"].as<double>();
        }
        if (config_node["param_residual_gate_dynamic_gain"]) {
            param_residual_gate_dynamic_gain_ = config_node["param_residual_gate_dynamic_gain"].as<double>();
        }

        // 2) 读取传感器级参数（量程、盲区、可选相机配置等）。
        YAML::Node sensor_config_node = YAML::LoadFile(sensor_config_file_);
        try {
            param_detect_range_ = sensor_config_node["param_detect_range"].as<float>();
            param_detect_blind_ = sensor_config_node["param_detect_blind"].as<float>();
        } catch (...) {
            LOG(FATAL) << "[Odometry]: file " << sensor_config_file_ << " has a bad conversion";
        }

#ifdef GEN_RGB_MAP
        try {
            param_enable_colorize_ = sensor_config_node["camera"]["enable_colorize"].as<bool>();
        } catch (...) {
            param_enable_colorize_ = false;
        }

        try {
            param_td_cam_relative_imu_ = sensor_config_node["param_td_cam_relative_imu"].as<double>();
        } catch (...) {
            param_td_cam_relative_imu_ = 0.0;
        }
#endif

        LOG(INFO) << std::endl
                  << "| --------------- [Odometry parameter] --------------- |" << std::endl
                  << "[param_detect_range_] " << param_detect_range_ << std::endl
                  << "[param_detect_blind_] " << param_detect_blind_ << std::endl
#ifdef GEN_RGB_MAP
                  << "[camera.enable_colorize] " << param_enable_colorize_ << std::endl
                  << "[param_td_cam_relative_imu] " << param_td_cam_relative_imu_ << std::endl
#endif
                  << "[param_flg_auto_imu_noise_analysis_] " << param_flg_auto_imu_noise_analysis_ << std::endl
                  << "[param_flg_auto_subframe_num_analysis_] " << param_flg_auto_subframe_num_analysis_ << std::endl
                  << "[param_min_subframe_num_] " << param_min_subframe_num_ << std::endl
                  << "[param_max_subframe_num_] " << param_max_subframe_num_ << std::endl
                  << "[param_max_imu_acc_noise_inc_] " << param_max_imu_acc_noise_inc_ << std::endl
                  << "[param_max_imu_gyr_noise_inc_] " << param_max_imu_gyr_noise_inc_ << std::endl
                  << "[param_high_dynamic_ratio_thres_] " << param_high_dynamic_ratio_thres_ << std::endl
                  << "[param_high_dynamic_q_scale_] " << param_high_dynamic_q_scale_ << std::endl
                  << "[param_flg_residual_gating_] " << param_flg_residual_gating_ << std::endl
                  << "[param_residual_gate_base_] " << param_residual_gate_base_ << std::endl
                  << "[param_residual_gate_range_scale_] " << param_residual_gate_range_scale_ << std::endl
                  << "[param_residual_gate_dynamic_gain_] " << param_residual_gate_dynamic_gain_ << std::endl
                  << "[param_flg_use_smooth_] " << param_flg_use_smooth_ << std::endl
                  << "[param_smooth_iteration_num_] " << param_smooth_iteration_num_ << std::endl
                  << "[param_thres_degrade_pos_] " << param_thres_degrade_pos_ << std::endl
                  << "[param_thres_degrade_pos_] " << param_thres_degrade_pos_ / M_PI * 180.0 << std::endl
                  << "[param_min_point_num_for_registration_] " << param_min_point_num_for_registration_ << std::endl
                  << "[param_min_point_num_for_downsample_] " << param_min_point_num_for_downsample_ << std::endl
                  << "[param_max_iteration_num_max_] " << param_max_iteration_num_max_ << std::endl
                  << "[param_max_rematch_num_max_] " << param_max_rematch_num_max_ << std::endl
                  << "[param_max_iteration_num_min_] " << param_max_iteration_num_min_ << std::endl
                  << "[param_max_rematch_num_min_] " << param_max_rematch_num_min_ << std::endl
                  << "[param_max_pt_searched_num_] " << param_max_pt_searched_num_ << std::endl
                  << "[param_min_pt_searched_num_] " << param_min_pt_searched_num_ << std::endl
                  << "[param_angle_converage_thres_] " << param_angle_converage_thres_ << std::endl
                  << "[param_position_converage_thres_] " << param_position_converage_thres_ << std::endl
                  << "[param_gravity_value_] " << param_gravity_value_ << std::endl
                  << "[param_point_filter_num_] " << param_point_filter_num_ << std::endl
                  << "[param_lidar_points_downsample_resolution_] " << param_lidar_points_downsample_resolution_ << std::endl
                  << "[param_flg_downsample_lidar_points_before_split_] " << param_flg_downsample_lidar_points_before_split_ << std::endl
                  << "[param_initialization_method_] " << param_initialization_method_ << std::endl
                  << "[param_static_initialize_time_] " << param_static_initialize_time_ << std::endl
                  << "[param_flg_estimate_imu_noise_] " << param_flg_estimate_imu_noise_ << std::endl
                  << "[param_pos_error_thres_] " << param_pos_error_thres_ << std::endl
                  << "[param_ang_error_thres_] " << param_ang_error_thres_ << std::endl
                  << std::endl;

        // 3) 初始化依赖模块参数。
        Imu_Integration::load_imu_noise(sensor_config_file_);
        ext_group_.load_config_parameter(sensor_config_file_);
        voxel_map_.load_config_parameter(system_dir_);

#ifdef GEN_RGB_MAP
        if (param_enable_colorize_) {
            cam_intrinsic_.load_config_parameter(sensor_config_file_);
        }
#endif
    }

    void add_image_data(std::shared_ptr<Image_Data> image_data)
    {
#ifdef GEN_RGB_MAP
        image_data->timestamp_ += param_td_cam_relative_imu_;
#endif
        sensor_synchronizer_.add_image_data(image_data);
    }

    void add_lidar_data(std::shared_ptr<Lidar_Data<PointType>> lidar_data)
    {
        sensor_synchronizer_.add_lidar_data(lidar_data);
    }

    void add_imu_data(std::shared_ptr<Imu_Data> imu_data)
    {
        sensor_synchronizer_.add_imu_data(imu_data);
    }

    void add_high_freq_pose_data(std::shared_ptr<Pose_Data> pose_data)
    {
        sensor_synchronizer_.add_high_freq_pose_data(pose_data);
    }

    bool run()
    {
        if (synchronized_data_detect()) {
            if (flg_need_init_) {
                if (system_initialize()) {
                    flg_need_init_ = false;
                }
            } else {
                Timer::evaluate("run", [&]() {
                    Timer::evaluate("split_lidar_points", [&, this]() { split_lidar_points(); });
                    Timer::evaluate("process_a_subframe", [&, this]() {
                        while (process_a_subframe()) {
                        }
                    });
                    Timer::evaluate("constraint_integrity_analysis", [&, this]() { constraint_integrity_analysis(); });
                    Timer::evaluate("build_map", [&, this]() { build_map(); });
                });
            }
            return true;
        } else {
            return false;
        }
    }

    bool synchronized_data_detect()
    {
        if (sensor_synchronizer_.lidar_imu_sync(lidar_image_imu_syn_meas_)) {
#ifdef GEN_RGB_MAP
            if (param_enable_colorize_) {
                latest_image_data_ = sensor_synchronizer_.get_latest_image_before(lidar_image_imu_syn_meas_->lidar_end_time_);
            }
#endif
#ifdef MY_DEBUG_MODE
            sensor_synchronizer_.print_syn_meas(lidar_image_imu_syn_meas_); // 打印当前同步好的数据
#endif
            return true;
        } else {
            return false;
        }
    }

    // 系统初始化，初始化结束（成功）返回true，否则返回false
    bool system_initialize()
    {
        switch (param_initialization_method_) {
        case 0:
            return system_static_initialize();
        default:
            LOG(FATAL) << "系统初始化方法未定义" << std::endl;
        }
        return false;
    }

    // 系统静态初始化，初始化结束（成功）返回true，否则返回false
    bool system_static_initialize()
    {
        // 1.系统初始状态估计、imu噪声参数估计
        {
            static double initialize_begin_time = lidar_image_imu_syn_meas_->lidar_data_->begin_time_;
            static std::vector<Eigen::Vector3d> acc_datas, gyr_datas;

            for (auto &imu : lidar_image_imu_syn_meas_->imu_datas_) {
                acc_datas.push_back(imu->acc_);
                gyr_datas.push_back(imu->gyr_);
            }

            double curr_time = lidar_image_imu_syn_meas_->lidar_data_->end_time_;
            if (curr_time - initialize_begin_time > param_static_initialize_time_) {
                Eigen::Vector3d acc_average, gyr_average;
                acc_average.setZero();
                for (auto &acc : acc_datas) {
                    acc_average += acc;
                }
                acc_average /= acc_datas.size();
                gyr_average.setZero();
                for (auto &gyr : gyr_datas) {
                    gyr_average += gyr;
                }
                gyr_average /= gyr_datas.size();

                State_Group init_state;
                init_state.timestamp_ = lidar_image_imu_syn_meas_->lidar_data_->end_time_;
                init_state.pos_ = Eigen::Vector3d::Zero();
                init_state.vel_ = Eigen::Vector3d::Zero();
                init_state.ba_ = Eigen::Vector3d::Zero();
                init_state.bg_ = gyr_average;
                init_state.grav_ = -abs(param_gravity_value_) * Eigen::Vector3d::UnitZ(); // 假设重力沿着Z轴负方向
                Eigen::Vector3d grav_in_imu_frame = -acc_average; // IMU坐标系下的重力
                init_state.quat_ = Eigen::Quaterniond::FromTwoVectors(grav_in_imu_frame / grav_in_imu_frame.norm(), init_state.grav_ / init_state.grav_.norm());
                init_state.cov_ = Eigen::Matrix<double, 18, 18>::Identity();
                Imu_Integration::set_init_state(init_state);

                if (param_flg_estimate_imu_noise_) {
                    Eigen::Vector3d acc_std, gyr_std;

                    acc_std.setZero();
                    for (auto &acc : acc_datas) {
                        for (int i = 0; i < 3; i++) {
                            acc_std[i] += (acc[i] - acc_average[i]) * (acc[i] - acc_average[i]);
                        }
                    }
                    for (int i = 0; i < 3; i++) {
                        acc_std[i] = sqrt(acc_std[i] / (acc_datas.size()));
                    }

                    gyr_std.setZero();
                    for (auto &gyr : gyr_datas) {
                        for (int i = 0; i < 3; i++) {
                            gyr_std[i] += (gyr[i] - gyr_average[i]) * (gyr[i] - gyr_average[i]);
                        }
                    }
                    for (int i = 0; i < 3; i++) {
                        gyr_std[i] = sqrt(gyr_std[i] / (gyr_datas.size()));
                    }

                    Imu_Integration::set_imu_noise_std_init(acc_std.sum() / 3.0, gyr_std.sum() / 3.0, 0.0, 0.0);
                }

                high_freq_traj_.emplace_back(Imu_Integration::get_curr_system_state());
                low_freq_traj_.emplace_back(Imu_Integration::get_curr_system_state());
            } else {
                return false;
            }
        }

        // 2.地图初始化
        // 2.1 读取当前IMU状态（系统初始化阶段前半部分已经给 Imu_Integration 设好了初值）
        State_Group imu_state = Imu_Integration::get_curr_system_state();
        // 2.2 由 IMU 位姿 + 外参，计算当前雷达在世界坐标系下的姿态和位置
        //     R_w_l = R_w_i * R_i_l
        //     t_w_l = R_w_i * t_i_l + t_w_i
        Eigen::Quaterniond lidar_state_quat = imu_state.quat_ * ext_group_.quat_i_l_;
        Eigen::Vector3d lidar_state_pos = imu_state.quat_ * ext_group_.trans_i_l_ + imu_state.pos_;

        // 2.3 对原始点云做“按序号均匀抽样”（每 param_point_filter_num_ 个点取1个）
        //     目的：快速降低点数，减少初始化建图开销
        typename pcl::PointCloud<PointType>::Ptr local_num_filter_points;
        if (param_point_filter_num_ != 1) {
            local_num_filter_points.reset(new typename pcl::PointCloud<PointType>());
            auto &pts_tmp = lidar_image_imu_syn_meas_->lidar_data_->cloud_->points;
            for (int i = 0; i < lidar_image_imu_syn_meas_->lidar_data_->cloud_->size(); i++) {
                if (i % param_point_filter_num_ == 0) {
                    local_num_filter_points->push_back(pts_tmp[i]);
                }
            }
        } else {
            // 步长为1时不做抽样，直接复用原始点云指针
            local_num_filter_points = lidar_image_imu_syn_meas_->lidar_data_->cloud_;
        }

        // 2.4 体素降采样（voxel filter），进一步控制点云密度
        //     分辨率由 param_lidar_points_downsample_resolution_ 指定
        typename pcl::PointCloud<PointType>::Ptr local_downsampled_points = Tools::downsample_voxel_filter<PointType>(local_num_filter_points, param_lidar_points_downsample_resolution_);

        // 2.5 将降采样后的点从雷达局部坐标系变换到全局坐标系（初始化建图坐标系）
        typename pcl::PointCloud<PointType>::Ptr global_downsampled_undistorted_points = local_downsampled_points;
        Eigen::Vector3d pt_eig;
        for (auto &pt_pcl : global_downsampled_undistorted_points->points) {
            // 局部点 p_l
            pt_eig << pt_pcl.x, pt_pcl.y, pt_pcl.z;
            // 全局点 p_w = R_w_l * p_l + t_w_l
            pt_eig = lidar_state_quat * pt_eig + lidar_state_pos;
            pt_pcl.x = pt_eig[0];
            pt_pcl.y = pt_eig[1];
            pt_pcl.z = pt_eig[2];
        }

        // 2.6 先根据当前雷达位置做局部地图裁剪（维持滑窗地图范围）
        voxel_map_.incremental_cut_map(lidar_state_pos[0], lidar_state_pos[1], lidar_state_pos[2]);
        // 2.7 将初始化帧点云加入体素地图，作为后续配准的初始地图
        voxel_map_.add_points_auto_downsample(global_downsampled_undistorted_points->points);

        return true;
    }

    void split_lidar_points()
    {
        // 1) 先做本帧自适应分析，更新：子帧数、IMU噪声、优化迭代参数等
        frame_split_analysis();

        // 2) 决定“是否在切分前先做整帧降采样”
        //    先取配置项，再根据规则动态修正（子帧过少时强制前置降采样）
        flg_downsample_lidar_points_before_split_ = param_flg_downsample_lidar_points_before_split_;
        typename pcl::PointCloud<PointType>::Ptr local_downsampled_points;
        if (flg_downsample_lidar_points_before_split_ || subframe_num_ <= 4) {
            // 子帧数很少时，为保证计算量可控，强制开启前置降采样
            flg_downsample_lidar_points_before_split_ = true;

            // 2.1 先做按序号均匀抽样（每 N 点取 1 点）
            typename pcl::PointCloud<PointType>::Ptr local_num_filter_points;
            if (param_point_filter_num_ != 1) {
                local_num_filter_points.reset(new typename pcl::PointCloud<PointType>());
                auto &pts_tmp = lidar_image_imu_syn_meas_->lidar_data_->cloud_->points;
                for (int i = 0; i < lidar_image_imu_syn_meas_->lidar_data_->cloud_->size(); i++) {
                    if (i % param_point_filter_num_ == 0) {
                        local_num_filter_points->push_back(pts_tmp[i]);
                    }
                }
            } else {
                // N=1 时不抽样，直接复用原始点云
                local_num_filter_points = lidar_image_imu_syn_meas_->lidar_data_->cloud_;
            }

            // 2.2 再做体素降采样，得到后续切分使用的点云
            local_downsampled_points = Tools::downsample_voxel_filter<PointType>(local_num_filter_points, param_lidar_points_downsample_resolution_);
        } else {
            // 不做前置降采样时，直接使用原始点云进入后续切分
            local_downsampled_points = lidar_image_imu_syn_meas_->lidar_data_->cloud_;
        }

        // 3) 按时间将激光帧切分为 subframe_num_ 个子帧
        std::vector<std::shared_ptr<Lidar_Data<PointType>>> lidar_datas(subframe_num_);
        if (subframe_num_ != 1) {
            // 当前激光帧起止时间
            double lidar_frame_begin_time = lidar_image_imu_syn_meas_->lidar_data_->begin_time_;
            double lidar_frame_end_time = lidar_image_imu_syn_meas_->lidar_data_->end_time_;
            // 每个子帧时间跨度
            double time_interval = (lidar_frame_end_time - lidar_frame_begin_time) / subframe_num_;
            int last_subframe_id = subframe_num_ - 1;

            // 为每个子帧创建 Lidar_Data 容器，并写入其时间边界
            for (int id = 0; id < subframe_num_; id++) {
                typename pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
                lidar_datas[id].reset(new Lidar_Data<PointType>(lidar_frame_begin_time + time_interval * (id), lidar_frame_begin_time + time_interval * (id + 1), cloud));
            }

            // 将每个点按时间戳分配到对应子帧
            size_t idx;
            for (auto &pt : local_downsampled_points->points) {
                idx = (pt.timestamp - lidar_frame_begin_time) / time_interval;
                // 边界保护：理论上不会小于0，但保留防御逻辑
                if (idx < 0) {
                    idx = 0;
                } else if (idx > last_subframe_id) {
                    idx = last_subframe_id;
                }
                lidar_datas[idx]->cloud_->push_back(pt);
            }
        } else {
            // 不切分时，整帧作为唯一子帧
            lidar_datas.front().reset(new Lidar_Data<PointType>(lidar_image_imu_syn_meas_->lidar_data_->begin_time_, lidar_image_imu_syn_meas_->lidar_data_->end_time_, local_downsampled_points));
        }

        // 4) 若未做前置降采样，则在“切分后”对子帧逐个预处理+降采样
        v_pts_used_.push_back(0);
        if (!flg_downsample_lidar_points_before_split_) {
            typename pcl::PointCloud<PointType>::Ptr points_tmp;
            for (int id = 0; id < lidar_datas.size(); id++) {
                // 先做距离裁剪（盲区/超量程剔除）
                lidar_datas[id]->cloud_ = lidar_points_preprocess(lidar_datas[id]->cloud_);
                // 点数足够时再做体素降采样，避免过小点云被过度稀疏化
                if (lidar_datas[id]->cloud_->size() > param_min_point_num_for_downsample_) {
                    points_tmp = Tools::downsample_voxel_filter<PointType>(lidar_datas[id]->cloud_, param_lidar_points_downsample_resolution_);
                    if (points_tmp->size() >= param_min_point_num_for_downsample_) {
                        lidar_datas[id]->cloud_ = points_tmp;
                    }
                }
                // 统计本帧最终参与后续处理的总点数
                v_pts_used_.back() += lidar_datas[id]->cloud_->size();
            }
        } else {
            // 已做前置降采样时，仅统计各子帧点数
            for (int id = 0; id < lidar_datas.size(); id++) {
                v_pts_used_.back() += lidar_datas[id]->cloud_->size();
            }
        }

        // 5) 将 IMU 数据按同样时间边界切分给每个子帧
        std::vector<std::deque<std::shared_ptr<Imu_Data>>> imu_datass(subframe_num_);
        if (subframe_num_ != 1) {
            // 保存子帧边界处插值得到的“桥接 IMU”，用于保证时间连续性
            std::shared_ptr<Imu_Data> tmp_imu_data;
            int imu_idx = 0;
            const int imu_size = static_cast<int>(lidar_image_imu_syn_meas_->imu_datas_.size());
            for (int id = 0; id < subframe_num_; id++) {
                std::deque<std::shared_ptr<Imu_Data>> &imu_datas = imu_datass[id];

                // 若上一子帧末尾构造过边界插值 IMU，则先放入当前子帧开头
                if (tmp_imu_data.use_count() != 0) {
                    imu_datas.push_back(tmp_imu_data);
                }

                // 收集时间戳落在当前子帧结束时刻之前的原始 IMU
                while (imu_idx < imu_size && lidar_image_imu_syn_meas_->imu_datas_[imu_idx]->timestamp_ < lidar_datas[id]->end_time_) {
                    imu_datas.push_back(lidar_image_imu_syn_meas_->imu_datas_[imu_idx]);
                    imu_idx++;
                }

                // 最后一个子帧：尝试补入剩余最后一个 IMU 样本
                if (id == int(lidar_datas.size()) - 1) {
                    if (imu_idx >= 0 && imu_idx < imu_size) {
                        imu_datas.push_back(lidar_image_imu_syn_meas_->imu_datas_[imu_idx]);
                    } else if (imu_size > 0) {
                        // 兜底：当 imu_idx 已越界时，补上最后一条有效 IMU，避免空段
                        imu_datas.push_back(lidar_image_imu_syn_meas_->imu_datas_[imu_size - 1]);
                    }
                } else {
                    // 非最后子帧：在子帧边界时刻做线性插值，生成一条边界 IMU
                    if (imu_idx > 0 && imu_idx < imu_size) {
                        double time = lidar_datas[id]->end_time_;
                        double t0 = lidar_image_imu_syn_meas_->imu_datas_[imu_idx - 1]->timestamp_;
                        double t1 = lidar_image_imu_syn_meas_->imu_datas_[imu_idx]->timestamp_;
                        double dt = t1 - t0;

                        if (dt > 1e-6) {
                            Eigen::Vector3d acc = lidar_image_imu_syn_meas_->imu_datas_[imu_idx]->acc_ * (time - t0) / dt + lidar_image_imu_syn_meas_->imu_datas_[imu_idx - 1]->acc_ * (t1 - time) / dt;
                            Eigen::Vector3d gyr = lidar_image_imu_syn_meas_->imu_datas_[imu_idx]->gyr_ * (time - t0) / dt + lidar_image_imu_syn_meas_->imu_datas_[imu_idx - 1]->gyr_ * (t1 - time) / dt;
                            tmp_imu_data.reset(new Imu_Data(time, acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]));
                        } else {
                            // 兜底：时间间隔异常时直接复用当前 IMU
                            tmp_imu_data = lidar_image_imu_syn_meas_->imu_datas_[imu_idx];
                        }
                        imu_datas.push_back(tmp_imu_data);
                    } else if (imu_size > 0) {
                        // 兜底：IMU 边界不足时复用最近可用值，防止索引越界
                        int safe_idx = std::max(0, std::min(imu_idx, imu_size - 1));
                        tmp_imu_data = lidar_image_imu_syn_meas_->imu_datas_[safe_idx];
                        imu_datas.push_back(tmp_imu_data);
                    }
                }
            }
        }

        // 6) 用切分后的 LiDAR + IMU 构造 Sub_Frame，压入待处理队列
        //    从当前队尾开始处理本轮新加子帧
        curr_frame_front_idx_ = subframes_.size();
        if (subframe_num_ != 1) {
            for (int id = 0; id < lidar_datas.size(); id++) {
                subframes_.emplace_back(new Sub_Frame<PointType>(lidar_datas[id], imu_datass[id]));
                // 为每个子帧分配全局递增 ID
                subframes_.back()->id_ = frame_id_count_++;
            }
        } else {
            subframes_.emplace_back(new Sub_Frame<PointType>(lidar_datas.front(), lidar_image_imu_syn_meas_->imu_datas_));
            subframes_.back()->id_ = frame_id_count_++;
        }

#ifdef GEN_RGB_MAP
        if (!subframes_.empty()) {
            subframes_.back()->set_image_data(latest_image_data_);
        }
#endif
    }

    typename pcl::PointCloud<PointType>::Ptr lidar_points_preprocess(typename pcl::PointCloud<PointType>::Ptr cloud)
    {
        typename pcl::PointCloud<PointType>::Ptr crop_cloud(new pcl::PointCloud<PointType>());
        crop_cloud->reserve(cloud->size());
        for (auto &pt : cloud->points) {
            if ((abs(pt.x) < param_detect_blind_ && abs(pt.y) < param_detect_blind_ && abs(pt.z) < param_detect_blind_) || abs(pt.x) > param_detect_range_ || abs(pt.y) > param_detect_range_ ||
                abs(pt.z) > param_detect_range_) {
                continue;
            }
            crop_cloud->push_back(pt);
        }
        return crop_cloud;
    }

    double clamp_unit_interval(double value) const
    {
        return std::max(0.0, std::min(1.0, value));
    }

    void update_dynamic_mode(double ratio)
    {
        // 将输入的运动强度比值限制到 [0, 1]，避免异常值影响后续自适应逻辑。
        motion_intensity_ratio_ = clamp_unit_interval(ratio);
        // 当运动强度超过阈值时，标记当前帧为高动态模式。
        high_dynamic_mode_ = (motion_intensity_ratio_ >= param_high_dynamic_ratio_thres_);
        // 每帧先将残差门控缩放重置为基准值 1.0。
        residual_gate_scale_ = 1.0;
        // 仅在高动态模式下对残差门控做额外放宽。
        if (high_dynamic_mode_) {
            // 按“动态增益 × 运动强度”线性增加门控缩放系数。
            residual_gate_scale_ += param_residual_gate_dynamic_gain_ * motion_intensity_ratio_;
        }
    }

    double get_residual_gate(const double point_norm) const
    {
        const double norm_term = std::max(0.0, point_norm);
        const double gate = param_residual_gate_base_ + param_residual_gate_range_scale_ * norm_term;
        return gate * residual_gate_scale_;
    }

    void frame_split_analysis()
    {
        // 读取上一时刻（上一帧末）系统状态，作为本次分析的积分起点
        State_Group last_frame_state = Imu_Integration::get_curr_system_state();
        // 当前同步测量内的 IMU 序列引用（避免拷贝）
        std::deque<std::shared_ptr<Imu_Data>> &imu_datas = lidar_image_imu_syn_meas_->imu_datas_;
        if (imu_datas.size() < 2) {
            update_dynamic_mode(0.0);
            subframe_num_ = param_min_subframe_num_;
            param_max_iteration_num_ = param_max_iteration_num_min_;
            param_max_rematch_num_ = param_max_rematch_num_min_;
            return;
        }

        // 说明：
        // 1) 角速度可由 IMU 观测减去陀螺零偏后直接得到；
        // 2) 线速度需要由去偏后的加速度积分得到；
        // 3) 这里用角速度/加速度/速度的波动程度衡量运动激烈程度，再自适应调整参数。
        // 保存每个时间步去偏后的角速度样本
        std::vector<Eigen::Vector3d> gyrs;
        // 保存每个时间步的无偏加速度（世界系）样本
        std::vector<Eigen::Vector3d> accs;
        // 保存积分得到的速度样本
        std::vector<Eigen::Vector3d> vels;
        // 临时变量：当前无偏加速度、梯形积分两端加速度、当前无偏角速度
        Eigen::Vector3d un_acc, un_acc_0, un_acc_1, un_gyr;
        // 上一时刻姿态（用于递推）
        Eigen::Quaterniond last_quat = last_frame_state.quat_;
        // 当前时刻姿态（由角速度积分得到）
        Eigen::Quaterniond curr_quat;
        // 当前速度（由加速度积分得到），初值取上一帧末速度
        Eigen::Vector3d curr_vel = last_frame_state.vel_;
        // 相邻 IMU 样本时间差
        double dt_tmp;

        // 先加入首个加速度样本：去偏后加上重力项（与后续积分量纲保持一致）
        accs.push_back((imu_datas.front()->acc_ - last_frame_state.ba_) + last_quat.inverse() * last_frame_state.grav_);
        // 从第 2 个 IMU 样本开始做递推积分
        for (int idx = 1; idx < imu_datas.size(); idx++) {
            // 当前步时间间隔
            dt_tmp = imu_datas[idx]->timestamp_ - imu_datas[idx - 1]->timestamp_;
            // //std::cout << "idx:" << idx << ", ";
            // //std::cout << "dt_tmp:" << dt_tmp << ", ";
            // 角速度去偏：两帧角速度均值（梯形）减去陀螺偏置
            un_gyr = 0.5 * (imu_datas[idx - 1]->gyr_ + imu_datas[idx]->gyr_) - last_frame_state.bg_;
            // 姿态传播：q_k = q_{k-1} * Exp(w*dt)
            curr_quat = last_quat * MathTools::Exp_Quat(un_gyr, dt_tmp);

            // 上一时刻去偏加速度，旋到世界系
            un_acc_0 = last_quat * (imu_datas[idx - 1]->acc_ - last_frame_state.ba_);
            // 当前时刻去偏加速度，旋到世界系
            un_acc_1 = curr_quat * (imu_datas[idx]->acc_ - last_frame_state.ba_);
            // 梯形法取平均并加重力，得到用于积分的线加速度
            un_acc = 0.5 * (un_acc_0 + un_acc_1) + last_frame_state.grav_;
            // 速度积分：v_k = v_{k-1} + a*dt
            curr_vel = curr_vel + un_acc * dt_tmp;
            // 更新“上一姿态”为当前姿态，供下一步递推
            last_quat = curr_quat;

            // 记录当前步样本，用于后续波动统计
            gyrs.push_back(un_gyr);
            accs.push_back(un_acc);
            vels.push_back(curr_vel);
        }

        // 统计中间变量：v3_tmp1 存均值，v3_tmp2 存方差累加
        Eigen::Vector3d v3_tmp1, v3_tmp2;
        // 三轴标准差
        Eigen::Vector3d std_gyr, std_acc, std_vel;
        // 分别记录三轴中的最大标准差（用于构造运动强度指标）
        double max_gyr_std = -1, max_acc_std = -1, max_vel_std = -1;
        // 初始化统计缓存
        v3_tmp1.setZero();
        v3_tmp2.setZero();
        // 计算角速度均值
        for (auto &gyr : gyrs) {
            v3_tmp1 += gyr;
        }
        v3_tmp1 /= gyrs.size();
        // 累计角速度方差
        for (auto &gyr : gyrs) {
            for (int i = 0; i < 3; i++) {
                v3_tmp2[i] += (gyr[i] - v3_tmp1[i]) * (gyr[i] - v3_tmp1[i]);
            }
        }
        // 角速度标准差，并取三轴最大值
        for (int i = 0; i < 3; i++) {
            std_gyr[i] = sqrt(v3_tmp2[i] / (gyrs.size()));
            if (std_gyr[i] > max_gyr_std) {
                max_gyr_std = std_gyr[i];
            }
        }

        // 开始统计加速度标准差
        v3_tmp1.setZero();
        v3_tmp2.setZero();
        // 计算加速度均值
        for (auto &acc : accs) {
            v3_tmp1 += acc;
        }
        v3_tmp1 /= accs.size();
        // 累计加速度方差
        for (auto &acc : accs) {
            for (int i = 0; i < 3; i++) {
                v3_tmp2[i] += (acc[i] - v3_tmp1[i]) * (acc[i] - v3_tmp1[i]);
            }
        }
        // 加速度标准差，并取三轴最大值
        for (int i = 0; i < 3; i++) {
            std_acc[i] = sqrt(v3_tmp2[i] / (accs.size()));
            if (std_acc[i] > max_acc_std) {
                max_acc_std = std_acc[i];
            }
        }

        // 开始统计速度标准差
        v3_tmp1.setZero();
        v3_tmp2.setZero();
        // 计算速度均值
        for (auto &vel : vels) {
            v3_tmp1 += vel;
        }
        v3_tmp1 /= vels.size();
        // 累计速度方差
        for (auto &vel : vels) {
            for (int i = 0; i < 3; i++) {
                v3_tmp2[i] += (vel[i] - v3_tmp1[i]) * (vel[i] - v3_tmp1[i]);
            }
        }
        // 速度标准差，并取三轴最大值
        for (int i = 0; i < 3; i++) {
            std_vel[i] = sqrt(v3_tmp2[i] / (vels.size()));
            if (std_vel[i] > max_vel_std) {
                max_vel_std = std_vel[i];
            }
        }
#ifdef MY_DEBUG_MODE
        // 调试模式下保存本帧统计量，便于离线分析
        max_acc_stds_.push_back(max_acc_std);
        max_gyr_stds_.push_back(max_gyr_std);
        max_vel_stds_.push_back(max_vel_std);
#endif

        // 将不同物理量归一化到同量纲，取最大值作为运动强度 ratio
        // 经验分母：30(加速度)、20(角速度)、25(速度)
        double ratio = max_acc_std / 30.0 > max_gyr_std / 20.0 ? max_acc_std / 30.0 : max_gyr_std / 20.0;
        ratio = max_vel_std / 25.0 > ratio ? max_vel_std / 25.0 : ratio;
        update_dynamic_mode(ratio);

        // 若开启 IMU 噪声自适应：在初值基础上按 ratio 线性增大
        if (param_flg_auto_imu_noise_analysis_) {
            // 高动态模式下对过程噪声做额外放大，降低过度依赖运动模型带来的累积漂移。
            double q_scale = 1.0;
            if (high_dynamic_mode_) {
                q_scale += param_high_dynamic_q_scale_ * motion_intensity_ratio_;
            }
            double noise_acc_std = Imu_Integration::get_imu_noise_acc_std_init() + param_max_imu_acc_noise_inc_ * motion_intensity_ratio_ * q_scale;
            double noise_gyr_std = Imu_Integration::get_imu_noise_gyr_std_init() + param_max_imu_gyr_noise_inc_ * motion_intensity_ratio_ * q_scale;
            // 写回当前帧生效的噪声参数
            Imu_Integration::set_imu_noise_std(noise_acc_std, noise_gyr_std, 0, 0);
        }

        // 若开启子帧数自适应：依据 ratio 在 [min, max] 之间插值
        if (param_flg_auto_subframe_num_analysis_) {
            subframe_num_ = param_min_subframe_num_ + (param_max_subframe_num_ - param_min_subframe_num_) * motion_intensity_ratio_;
            // 限幅，防止越界
            if (subframe_num_ > param_max_subframe_num_) {
                subframe_num_ = param_max_subframe_num_;
            } else if (subframe_num_ < param_min_subframe_num_) {
                subframe_num_ = param_min_subframe_num_;
            }
            // 同步自适应优化迭代上限与 rematch 次数阈值
            param_max_iteration_num_ = param_max_iteration_num_min_ + motion_intensity_ratio_ * (param_max_iteration_num_max_ - param_max_iteration_num_min_);
            param_max_rematch_num_ = param_max_rematch_num_min_ + motion_intensity_ratio_ * (param_max_rematch_num_max_ - param_max_rematch_num_min_);

            // 触发高动态后，进一步放宽优化预算以提高收敛稳定性。
            if (high_dynamic_mode_) {
                param_max_iteration_num_ = std::min(param_max_iteration_num_max_, param_max_iteration_num_ + 2);
                param_max_rematch_num_ = std::min(param_max_rematch_num_max_, param_max_rematch_num_ + 1);
            }
        } else {
            // 关闭自适应时，全部回落到固定最小配置
            subframe_num_ = param_min_subframe_num_;
            param_max_iteration_num_ = param_max_iteration_num_min_;
            param_max_rematch_num_ = param_max_rematch_num_min_;
        }

        // 记录本帧最终采用的参数，供日志和后处理使用
        subframe_nums_.push_back(subframe_num_);
        imu_acc_noise_stds_.push_back(Imu_Integration::get_imu_noise_acc_std());
        imu_gyr_noise_stds_.push_back(Imu_Integration::get_imu_noise_gyr_std());
    }

    bool process_a_subframe()
    {
        // 仅当仍有未处理子帧时才执行处理；否则返回 false 表示本轮结束
        if (curr_frame_front_idx_ < subframes_.size()) {
            // 取出当前待处理子帧，并将前指针后移
            std::shared_ptr<Sub_Frame<PointType>> curr_sub_frame = subframes_[curr_frame_front_idx_++];
            // 标记该子帧已进入估计流程
            curr_sub_frame->flg_has_estimated_ = true;
            // 先进行一次预测传播（仅 IMU 传播，不含激光更新）
            curr_sub_frame->get_const_speed_eskf()->propagate();
            // 记录该子帧最终迭代次数（后续用于统计）
            int iteration_count = 0;

            // 若点数过少，直接跳过配准，仅保留空的去畸变点云占位
            if (curr_sub_frame->get_lidar_data()->cloud_->size() < param_min_point_num_for_registration_) {
                typename pcl::PointCloud<PointType>::Ptr local_undistorted_cloud(new pcl::PointCloud<PointType>());
                curr_sub_frame->set_local_undistorted_points(local_undistorted_cloud);
                curr_sub_frame->flg_use_for_registration_ = false;

                // 无有效配准时，统计项置 0
                v_opt_iter_counts_.push_back(0);
                v_rematch_num_.push_back(0);
            } else {
                // 正常配准分支
                // flg_enable_nearest_search 控制本轮是否重新做邻域搜索
                bool flg_enable_nearest_search = true;
                // 预留标志：配准是否成功
                bool flg_register_success = true;
                // flg_ieskf_converged：当前轮 IEKF 是否收敛
                // flg_ieskf_converged_last_iter：上一轮是否收敛
                // flg_ieskf_stop：是否满足停止条件
                bool flg_ieskf_converged, flg_ieskf_converged_last_iter, flg_ieskf_stop = false;
                // 有效残差数，rematch 次数（初始记为 1）
                int vaild_residual_num, rematch_num = 1;
                typename pcl::PointCloud<PointType>::Ptr local_undistorted_cloud;

                // 进入迭代优化（硬上限为 param_max_iteration_num_max_）
                for (; iteration_count < param_max_iteration_num_max_; iteration_count++) {
                    // 首次迭代先做运动补偿得到局部去畸变点云
                    if (iteration_count == 0 /* flg_enable_nearest_search */) {
                        Lidar_Motion_Compensate_Tool lidar_motion_compensate_tool_tmp(curr_sub_frame->get_const_speed_eskf()->get_high_freq_states(), ext_group_.quat_i_l_, ext_group_.trans_i_l_);
                        local_undistorted_cloud = lidar_motion_compensate_tool_tmp.motion_compensation_for_one_frame<PointType>(
                            curr_sub_frame->get_lidar_data()->cloud_, curr_sub_frame->get_const_speed_eskf()->get_curr_state().quat_, curr_sub_frame->get_const_speed_eskf()->get_curr_state().pos_);
                        // 保存子帧去畸变点云，供后续建图阶段复用
                        curr_sub_frame->set_local_undistorted_points(local_undistorted_cloud);

#if MY_DEBUG_MODE
                        // 调试可视化：把去畸变点云投到世界系
                        Eigen::Vector3d pt_eig;
                        PointType pt_pcl;
                        predict_undistorted_cloud_->clear();
                        for (auto &pt : local_undistorted_cloud->points) {
                            pt_eig << pt.x, pt.y, pt.z;
                            pt_eig = curr_sub_frame->get_const_speed_eskf()->get_curr_state().quat_ * (ext_group_.quat_i_l_ * pt_eig + ext_group_.trans_i_l_) +
                                     curr_sub_frame->get_const_speed_eskf()->get_curr_state().pos_;
                            pt_pcl.x = pt_eig[0];
                            pt_pcl.y = pt_eig[1];
                            pt_pcl.z = pt_eig[2];
                            predict_undistorted_cloud_->push_back(pt_pcl);
                        }

                        // 调试可视化：把原始畸变点云也投到世界系对比
                        predict_distorted_cloud_->clear();
                        for (auto &pt : curr_sub_frame->get_lidar_data()->cloud_->points) {
                            pt_eig << pt.x, pt.y, pt.z;
                            pt_eig = curr_sub_frame->get_const_speed_eskf()->get_curr_state().quat_ * (ext_group_.quat_i_l_ * pt_eig + ext_group_.trans_i_l_) +
                                     curr_sub_frame->get_const_speed_eskf()->get_curr_state().pos_;
                            pt_pcl.x = pt_eig[0];
                            pt_pcl.y = pt_eig[1];
                            pt_pcl.z = pt_eig[2];
                            predict_distorted_cloud_->push_back(pt_pcl);
                        }
#endif
                    }

                    // 数据关联：根据当前状态把点与地图建立约束
                    // 若开启计时，统计 data_associate 耗时
                    if (flg_enable_nearest_search) {
                        Timer::evaluate("data_associate", [&, this]() {
                            vaild_residual_num = data_associate_fastlio2(local_undistorted_cloud, curr_sub_frame->get_const_speed_eskf()->get_curr_state(), flg_enable_nearest_search);
                        });
                    } else {
                        vaild_residual_num = data_associate_fastlio2(local_undistorted_cloud, curr_sub_frame->get_const_speed_eskf()->get_curr_state(), flg_enable_nearest_search);
                    }

                    // 依据有效残差点数判断该子帧是否可用于配准
                    curr_sub_frame->flg_use_for_registration_ = (vaild_residual_num >= param_min_point_num_for_registration_);
                    if (!curr_sub_frame->flg_use_for_registration_) {
                        // 有效约束不足，直接终止当前子帧优化
                        v_opt_iter_counts_.push_back(iteration_count);
                        v_rematch_num_.push_back(rematch_num);

                        // 预先填充 closest_map_pts（全部置为无效），保持后续流程数据结构一致
                        auto &closest_map_pts = curr_sub_frame->closest_map_pts;
                        PointType pt_tmp;
                        closest_map_pts.resize(local_undistorted_cloud->size(), std::pair<bool, PointType>(false, pt_tmp));

                        flg_register_success = false;
                        break;
                    }

                    // 有足够约束时，构建 IEKF 更新所需 Jacobian 与残差
                    {
                        // 当前迭代状态
                        State_Group imu_state = curr_sub_frame->get_const_speed_eskf()->get_curr_state();
                        Eigen::Matrix3d imu_state_quat(imu_state.quat_.toRotationMatrix());
                        Eigen::Vector3d imu_state_pos(imu_state.pos_);
                        // IMU->LiDAR 外参旋转
                        Eigen::Matrix3d ext_r_i2l(ext_group_.quat_i_l_);
                        // Hsub: 残差对 [pos(3), rot(3)] 的雅可比
                        Eigen::MatrixXd Hsub(vaild_residual_num, 6); // Hsub(n x 6)
                        // 每个残差的观测方差（这里使用常量）
                        Eigen::MatrixXd Cov_i(vaild_residual_num, 1);
                        // 残差向量
                        Eigen::VectorXd res_vec(vaild_residual_num);
                        Eigen::Vector3d plane_normal;
                        Hsub.setZero();
                        Cov_i.setZero();
                        int residual_num = 0;
                        int pt_idx = 0;

                        // 从关联结果中提取有效约束，逐项填充 H 和 r
                        Timer::evaluate("calculate_residual_jacobian", [&, this]() {
                            for (auto &pair : associated_pairs_) {
                                if (pair.flg_valid_) {
                                    PointType &local_point_pcl = local_undistorted_cloud->points[pt_idx];
                                    plane_normal << pair.plane_coef_[0], pair.plane_coef_[1], pair.plane_coef_[2];
                                    Hsub.block<1, 3>(residual_num, 0) = plane_normal.transpose();
                                    Hsub.block<1, 3>(residual_num, 3) =
                                        -plane_normal.transpose() * imu_state_quat *
                                        MathTools::SkewSymmetric(ext_r_i2l * Eigen::Vector3d(local_point_pcl.x, local_point_pcl.y, local_point_pcl.z) + ext_group_.trans_i_l_);
                                    res_vec(residual_num) = pair.residual_;
                                    Cov_i(residual_num, 0) = 1.0 / (0.001);
                                    residual_num++;
                                }
                                // 已收集足够有效约束则提前退出
                                if (residual_num == vaild_residual_num) {
                                    break;
                                }
                                pt_idx++;
                            }
                        });

                        // IEKF 一步更新，返回是否收敛
                        Timer::evaluate("ieskf_update", [&, this]() {
                            flg_ieskf_converged =
                                curr_sub_frame->get_const_speed_eskf()->ieskf_update_state(Hsub, res_vec, Cov_i, vaild_residual_num, param_position_converage_thres_, param_angle_converage_thres_);
                        });

                        // Rematch 判断：
                        // 1) 当前收敛，触发重新搜索邻域；
                        // 2) 或者到达倒数第二轮时，强制再做一次重匹配。
                        if (flg_ieskf_converged || ((rematch_num == 1) && (iteration_count == (param_max_iteration_num_ - 2)))) {
                            // curr_sub_frame->get_const_speed_eskf()->smooth();
                            flg_enable_nearest_search = true;
                            rematch_num++;
                        } else {
                            // 不重匹配时，下一轮复用当前关联关系以节省计算
                            flg_enable_nearest_search = false;
                        }

                        // 停止条件与最终协方差更新：满足以下任一条件即可停止
                        // 1) rematch 次数超过阈值；
                        // 2) 已到目标迭代最后一轮；
                        // 3) 连续两轮（跨 rematch）都判定收敛。
                        if (!flg_ieskf_stop &&
                            (rematch_num > param_max_rematch_num_ || (iteration_count == param_max_iteration_num_ - 1) || (rematch_num > 1 && flg_ieskf_converged && flg_ieskf_converged_last_iter))) {
                            curr_sub_frame->get_const_speed_eskf()->ieskf_update_cov_finally();
                            flg_ieskf_stop = true;
                        }

                        // 保存当前轮收敛状态，供下一轮停止判据使用
                        flg_ieskf_converged_last_iter = flg_ieskf_converged;
                    }

                    // 若触发停止，落统计并缓存每个点最近地图点（供建图判重）
                    if (flg_ieskf_stop) {
                        v_opt_iter_counts_.push_back(iteration_count);
                        v_rematch_num_.push_back(rematch_num);

                        auto &closest_map_pts = curr_sub_frame->closest_map_pts;
                        closest_map_pts.reserve(local_undistorted_cloud->size());
                        PointType pt_tmp;
                        int pt_idx = 0;
                        for (auto &pair : associated_pairs_) {
                            auto &points_near = pair.nearest_points_;
                            // 若存在近邻，记录首个最近点；否则标记无效
                            if (points_near.size() > 0) {
                                closest_map_pts.emplace_back(true, points_near[0]);
                            } else {
                                closest_map_pts.emplace_back(false, pt_tmp);
                            }
                            pt_idx++;
                            // 与 local_undistorted_cloud 对齐，达到点数后终止
                            if (pt_idx == local_undistorted_cloud->size()) {
                                break;
                            }
                        }
                        // 当前子帧处理完成，退出迭代
                        break;
                    }
                }
            }

            // 成功处理了一个子帧（无论是否用于配准）
            return true;
        } else {
            // 当前没有可处理子帧
            return false;
        }
    }

    int data_associate_fastlio2(typename pcl::PointCloud<PointType>::Ptr local_undistorted_cloud, const State_Group &imu_state, bool flg_coverage = true)
    {
        // 由 IMU 位姿和 IMU->LiDAR 外参计算当前雷达姿态（世界系）。
        Eigen::Quaterniond lidar_state_quat = imu_state.quat_ * ext_group_.quat_i_l_;
        // 由 IMU 位姿和 IMU->LiDAR 外参计算当前雷达位置（世界系）。
        Eigen::Vector3d lidar_state_pos = imu_state.quat_ * ext_group_.trans_i_l_ + imu_state.pos_;
        // std::cout << "data_associate_fastlio2 1" << std::endl;
        // 确保关联缓存容量不小于当前点数，避免后续按索引写入越界。
        if (associated_pairs_.size() < local_undistorted_cloud->size()) {
            // 扩容关联结果数组，每个点对应一个 Associated_Pair。
            associated_pairs_.resize(local_undistorted_cloud->size());
        }
        // std::cout << "data_associate_fastlio2 2" << std::endl;
        // 统计本次关联中通过有效性判定的残差数量。
        int vaild_residual_num = 0;
        // 中间变量：平面参数、点坐标（局部/全局）。
        Eigen::Vector3d plane_center, plane_normal, local_pt_eigen, global_pt_eigen;
        // 中间变量：PCL 格式点（局部/全局）。
        PointType local_pt_pcl, global_pt_pcl;
#if USE_FEATURE_MAP

#else
        // 保存最近邻查询返回的体素种子指针。
        Point_Seed<Dist_Node> *voxel_ptr;
        // 逐点进行地图关联与残差构建。
        for (int idx = 0; idx < local_undistorted_cloud->size(); idx++) {
            // 取当前局部去畸变点（雷达坐标系）。
            PointType &local_point_pcl = local_undistorted_cloud->points[idx];
            // 转为 Eigen 向量，便于矩阵运算。
            local_pt_eigen << local_point_pcl.x, local_point_pcl.y, local_point_pcl.z;
            // 将局部点变换到世界系，得到用于关联的全局点。
            global_pt_eigen = lidar_state_quat * local_pt_eigen + lidar_state_pos;
            // 取当前点对应的关联结构体引用。
            auto &pair = associated_pairs_[idx];
            // 取当前点关联到的近邻点容器引用。
            auto &points_near = pair.nearest_points_;

            // 当允许覆盖更新时，重新执行邻域搜索与平面估计。
            if (flg_coverage) {
                // 将全局点写回 PCL 结构，供地图查询接口使用。
                global_pt_pcl.x = global_pt_eigen[0];
                // 写入全局 y 坐标。
                global_pt_pcl.y = global_pt_eigen[1];
                // 写入全局 z 坐标。
                global_pt_pcl.z = global_pt_eigen[2];

                // 查询当前点在地图中的近邻点及其所属体素。
                voxel_map_.get_closest_point(global_pt_pcl, points_near, voxel_ptr);

                // 若命中有效体素指针，则继续平面约束判定。
                if (voxel_ptr) {
                    // 体素已缓存过平面补丁时，直接复用以降低重复估计开销。
                    if (voxel_ptr->get_flg_has_patch()) {
                        // 标记当前点可用于平面残差计算。
                        pair.flg_valid1_ = true;
                        // 读取体素缓存的平面系数。
                        pair.plane_coef_ = voxel_ptr->get_patch();
                    } else {
                        // 先检查近邻数量是否满足最小平面拟合要求。
                        pair.flg_valid1_ = points_near.size() >= param_min_pt_searched_num_;
                        // 近邻数量足够时再执行平面拟合。
                        if (pair.flg_valid1_) {
                            // 用近邻点估计局部平面，成功则返回平面参数。
                            pair.flg_valid1_ = Tools::esti_plane(pair.plane_coef_, points_near, 0.1f);
                            // 若拟合成功，将结果写回体素缓存供后续复用。
                            if (pair.flg_valid1_) {
                                // 标记该体素已有可复用平面补丁。
                                voxel_ptr->set_flg_has_patch();
                                // 缓存当前估计得到的平面系数。
                                voxel_ptr->set_patch(pair.plane_coef_);
                            }
                        }
                    }
                } else {
                    // 未命中体素时，当前点无法建立有效平面约束。
                    pair.flg_valid1_ = false;
                }
            }

            // 仅当存在有效平面时才继续计算点面残差。
            if (pair.flg_valid1_) {
                // 提取平面法向量前三项。
                plane_normal << pair.plane_coef_[0], pair.plane_coef_[1], pair.plane_coef_[2];
                // 计算点到平面的有符号距离（点面残差）。
                float pd2 = plane_normal.dot(global_pt_eigen) + pair.plane_coef_(3, 0);

                // 若开启残差门控，则先做阈值过滤。
                if (param_flg_residual_gating_) {
                    // 依据点距自适应计算当前点门控阈值。
                    const double residual_gate = get_residual_gate(local_pt_eigen.norm());
                    // 残差超过门限则判为无效并跳过本点。
                    if (std::abs(pd2) > residual_gate) {
                        // 标记该点残差无效。
                        pair.flg_valid_ = false;
                        // 跳过该点后续有效残差计数与写入。
                        continue;
                    }
                }

                // 有效残差数量加一。
                vaild_residual_num++;
                // 记录当前点面残差值。
                pair.residual_ = pd2;
                // 标记该点为有效关联。
                pair.flg_valid_ = true;
            } else {
                // 没有有效平面时，显式标记该点关联无效。
                pair.flg_valid_ = false;
            }
        }
#endif
        // 返回本轮可用于优化的有效残差点数量。
        return vaild_residual_num;
    }

    void constraint_integrity_analysis()
    {
        // 1.进行细分帧的平滑操作（subframes_中所有细分帧都是需要进行平滑的）
        if (param_flg_use_smooth_) {
            for (int id = subframes_.size() - 1; id >= 0; id--) {
                subframes_[id]->get_const_speed_eskf()->smooth();
                subframes_[id]->smooth_count_++;
            }
        }
        // 2.依次判断细分帧是否完成平滑
        if (param_flg_use_smooth_) {
            while (!subframes_.empty()) {
                if (subframes_.front()->smooth_count_ < param_smooth_iteration_num_ && (subframes_.front()->get_lidar_data()->end_time_ - subframes_.front()->get_lidar_data()->begin_time_) < 0.0333) {
                    break;
                } else {
                    subframes_.front()->flg_degrad_ = false;
                    subframes_waiting_to_add_to_map_.push_back(subframes_.front());
                    subframes_.pop_front();
                }
            }
        } else { // 不进行平滑时将本轮处理的所有细分帧均送入建图模块
            while (!subframes_.empty()) {
                subframes_.front()->flg_degrad_ = false;
                subframes_waiting_to_add_to_map_.push_back(subframes_.front());
                subframes_.pop_front();
            }
        }
    }

    void build_map()
    {
        // 1.攒够一定数量的细分帧后，进行建图操作
        double time_interval = subframes_waiting_to_add_to_map_.empty() ?
                                   -1 :
                                   (subframes_waiting_to_add_to_map_.back()->get_lidar_data()->end_time_ - subframes_waiting_to_add_to_map_.front()->get_lidar_data()->begin_time_);
        if (time_interval > 0) {
            int pts_num = 0;
            for (auto frame : subframes_waiting_to_add_to_map_) {
                pts_num += frame->get_lidar_data()->cloud_->size();
            }

            typename pcl::PointCloud<PointType>::Ptr global_undistorted_cloud(new pcl::PointCloud<PointType>());
            global_undistorted_cloud->reserve(pts_num);

#if MY_VIS_MODE
            map_cloud_tmp_->clear();
#endif
            if (0 /* param_flg_use_smooth_ */) {
                for (auto frame : subframes_waiting_to_add_to_map_) {
                    Lidar_Motion_Compensate_Tool lidar_motion_compensate_tool_tmp(frame->get_const_speed_eskf()->get_high_freq_states(), ext_group_.quat_i_l_, ext_group_.trans_i_l_);
                    lidar_motion_compensate_tool_tmp.motion_compensation_for_one_frame_proj_to_map<PointType>(frame->get_lidar_data()->cloud_, global_undistorted_cloud);
                }
            } else {
                double map_res_sqr;
                map_res_sqr = 0.95 * 0.95 * voxel_map_.get_point_resolution() * voxel_map_.get_point_resolution();
                for (auto frame : subframes_waiting_to_add_to_map_) {
                    Eigen::Quaterniond lidar_quat = frame->get_const_speed_eskf()->get_curr_state().quat_ * ext_group_.quat_i_l_;
                    Eigen::Vector3d lidar_pos = frame->get_const_speed_eskf()->get_curr_state().quat_ * ext_group_.trans_i_l_ + frame->get_const_speed_eskf()->get_curr_state().pos_;
                    Eigen::Vector3d pt_eigen;
                    int pt_idx = 0;
#ifdef GEN_RGB_MAP
                    typename pcl::PointCloud<PointType>::Ptr global_undistorted_cloud_tmp(new pcl::PointCloud<PointType>());
                    global_undistorted_cloud_tmp->reserve(frame->get_local_undistorted_points()->size());
#endif
                    for (auto &pt : frame->get_local_undistorted_points()->points) { // 一次使用，后续不能再用
                        pt_eigen << pt.x, pt.y, pt.z;
                        pt_eigen = lidar_quat * pt_eigen + lidar_pos;
                        pt.x = pt_eigen[0];
                        pt.y = pt_eigen[1];
                        pt.z = pt_eigen[2];
#if MY_VIS_MODE
                        map_cloud_tmp_->push_back(pt);
#endif
                        if (pt_idx < frame->closest_map_pts.size() && frame->closest_map_pts[pt_idx].first) {
                            auto &closest_map_pt = frame->closest_map_pts[pt_idx].second;
                            if (((closest_map_pt.x - pt.x) * (closest_map_pt.x - pt.x) + (closest_map_pt.y - pt.y) * (closest_map_pt.y - pt.y) +
                                 (closest_map_pt.z - pt.z) * (closest_map_pt.z - pt.z)) > map_res_sqr) {
                                global_undistorted_cloud->push_back(pt);
                            }
                        } else {
                            global_undistorted_cloud->push_back(pt);
                        }

#ifdef GEN_RGB_MAP
                        global_undistorted_cloud_tmp->push_back(pt);
#endif

                        pt_idx++;
                    }

#ifdef GEN_RGB_MAP
                    if (param_enable_colorize_) {
                        Eigen::Quaterniond camera_quat = frame->get_const_speed_eskf()->get_curr_state().quat_ * ext_group_.quat_i_c_;
                        Eigen::Vector3d camera_pos = frame->get_const_speed_eskf()->get_curr_state().quat_ * ext_group_.trans_i_c_ + frame->get_const_speed_eskf()->get_curr_state().pos_;

                        if (frame->get_image_data().use_count() != 0 && !frame->get_image_data()->img_.empty()) {
                            rbg_keyframe_gen_.add_frame(frame->get_timestamp(), camera_quat.toRotationMatrix(), camera_pos, global_undistorted_cloud_tmp, frame->get_image_data()->img_);
                        } else {
                            rbg_keyframe_gen_.add_frame(frame->get_timestamp(), camera_quat.toRotationMatrix(), camera_pos, global_undistorted_cloud_tmp, cv::Mat());
                        }
                    }
#endif
                }
            }

            Eigen::Vector3d curr_lidar_pos = subframes_waiting_to_add_to_map_.back()->get_const_speed_eskf()->get_curr_state().quat_ * ext_group_.trans_i_l_ +
                                             subframes_waiting_to_add_to_map_.back()->get_const_speed_eskf()->get_curr_state().pos_;

            voxel_map_.incremental_cut_map(curr_lidar_pos[0], curr_lidar_pos[1], curr_lidar_pos[2]);
            voxel_map_.add_points_auto_downsample(global_undistorted_cloud->points);

            for (auto frame : subframes_waiting_to_add_to_map_) {
                const auto &high_freq_states = frame->get_const_speed_eskf()->get_high_freq_states();
                for (int idx = 0; idx < high_freq_states.size(); idx++) {
                    high_freq_traj_.emplace_back(high_freq_states[idx]);
                }
                if (!high_freq_states.empty()) {
                    low_freq_traj_.emplace_back(high_freq_states.back());
                }
            }

            subframes_waiting_to_add_to_map_.clear();
        }
    }

#ifdef MY_DEBUG_MODE
    void save_imu_info(const std::string &file)
    {
        std::ofstream ofs;
        ofs.open(file, std::ios::out);
        if (!ofs.is_open()) {
            LOG(ERROR) << "Failed to open file: " << file;
            return;
        }

        ofs << "acc,gyr,vel,acc_std,gyr_std,vel_std,subframe_num" << std::endl;
        for (int idx = 0; idx < max_accs_.size(); idx++) {
            ofs << std::fixed << std::setprecision(6) << max_accs_[idx] << "," << max_gyrs_[idx] << "," << max_vels_[idx] << "," << max_acc_stds_[idx] << "," << max_gyr_stds_[idx] << ","
                << max_vel_stds_[idx] << "," << subframe_nums_[idx] << std::endl;
        }

        ofs.close();
    }
    void save_dx_info(const std::string &file)
    {
        std::ofstream ofs;
        ofs.open(file, std::ios::out);
        if (!ofs.is_open()) {
            LOG(ERROR) << "Failed to open file: " << file;
            return;
        }

        ofs << "pos,vel,ang" << std::endl;
        for (int idx = 0; idx < v_dx_pos_.size(); idx++) {
            ofs << std::fixed << std::setprecision(6) << v_dx_pos_[idx] << "," << v_dx_vel_[idx] << "," << v_dx_ang_[idx] << std::endl;
        }

        ofs.close();
    }
#endif

    void set_flg_need_init(bool flg_need_init)
    {
        flg_need_init_ = flg_need_init;
    }

    bool get_flg_need_init()
    {
        return flg_need_init_;
    }

    Extrinsic_Group &get_ext_group()
    {
        return ext_group_;
    }

    std::deque<std::shared_ptr<Sub_Frame<PointType>>> &get_subframes()
    {
        return subframes_;
    }
    std::vector<std::shared_ptr<Sub_Frame<PointType>>> &get_subframes_waiting_to_add_to_map()
    {
        return subframes_waiting_to_add_to_map_;
    }

    void print_all()
    {
        unsigned long long total_pt_num = 0;
        for (auto pt_num : v_pts_used_) {
            total_pt_num += pt_num;
        }
        LOG(INFO) << "mean points used per frame: " << total_pt_num / v_pts_used_.size() << std::endl;
    }

    void dump_into_file(const std::string &low_traj_file, const std::string &high_traj_file, const std::string &state_file = std::string())
    {
        for (auto frame : subframes_waiting_to_add_to_map_) {
            const auto &high_freq_states = frame->get_const_speed_eskf()->get_high_freq_states();
            for (int idx = 0; idx < high_freq_states.size(); idx++) {
                high_freq_traj_.emplace_back(high_freq_states[idx]);
            }
            if (!high_freq_states.empty()) {
                low_freq_traj_.emplace_back(high_freq_states.back());
            }
        }

        {
            std::ofstream ofs;
            ofs.open(low_traj_file, std::ios::out);
            if (!ofs.is_open()) {
                LOG(ERROR) << "Failed to open low_traj_file: " << low_traj_file;
            } else {
                ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
                for (const auto &state : low_freq_traj_) {
                    ofs << std::fixed << std::setprecision(6) << state.timestamp_ << " " << std::setprecision(15) << state.pos_.x() << " " << state.pos_.y() << " " << state.pos_.z() << " "
                        << state.quat_.x() << " " << state.quat_.y() << " " << state.quat_.z() << " " << state.quat_.w() << std::endl;
                }
                ofs.close();
            }
        }

        {
            std::ofstream ofs;
            ofs.open(high_traj_file, std::ios::out);
            if (!ofs.is_open()) {
                LOG(ERROR) << "Failed to open high_traj_file: " << high_traj_file;
            } else {
                ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
                for (const auto &state : high_freq_traj_) {
                    ofs << std::fixed << std::setprecision(6) << state.timestamp_ << " " << std::setprecision(15) << state.pos_.x() << " " << state.pos_.y() << " " << state.pos_.z() << " "
                        << state.quat_.x() << " " << state.quat_.y() << " " << state.quat_.z() << " " << state.quat_.w() << std::endl;
                }
                ofs.close();
            }
        }

        if (!state_file.empty()) {
            std::ofstream ofs;
            ofs.open(state_file, std::ios::out);
            if (!ofs.is_open()) {
                LOG(ERROR) << "Failed to open state_file: " << state_file;
                return;
            } else {
                ofs << "#timestamp x y z q_x q_y q_z q_w v_x v_y v_z ba_x ba_y ba_z bg_x bg_y bg_z g_x g_y g_z" << std::endl;
                for (const auto &state : high_freq_traj_) {
                    ofs << std::fixed << std::setprecision(6) << state.timestamp_ << " " << std::setprecision(15) << state.pos_.x() << " " << state.pos_.y() << " " << state.pos_.z() << " "
                        << state.quat_.x() << " " << state.quat_.y() << " " << state.quat_.z() << " " << state.quat_.w() << " " << state.vel_.x() << " " << state.vel_.y() << " " << state.vel_.z()
                        << " " << state.ba_.x() << " " << state.ba_.y() << " " << state.ba_.z() << " " << state.bg_.x() << " " << state.bg_.y() << " " << state.bg_.z() << " " << state.grav_.x() << " "
                        << state.grav_.y() << " " << state.grav_.z() << std::endl;
                }
                ofs.close();
            }
        }
    }

    void save_info(const std::string &file)
    {
        std::ofstream ofs;
        ofs.open(file, std::ios::out);
        if (!ofs.is_open()) {
            LOG(ERROR) << "Failed to open file: " << file;
            return;
        }

        ofs << "subframe_nums,imu_acc_noise_std,imu_gyr_noise_std,pts_used,opt_iter_count,rematch_num" << std::endl;
        for (int idx = 0; idx < subframe_nums_.size(); idx++) {
            ofs << std::fixed << std::setprecision(6) << subframe_nums_[idx] << "," << imu_acc_noise_stds_[idx] << "," << imu_gyr_noise_stds_[idx] << "," << v_pts_used_[idx] << ","
                << v_opt_iter_counts_[idx] << "," << v_rematch_num_[idx] << std::endl;
        }

        ofs.close();
    }

    void get_final_map_cloud(typename pcl::PointCloud<pcl::PointXYZI>::Ptr &map_cloud)
    {
        if (!map_cloud) {
            map_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        }
        map_cloud->clear();

        std::vector<Voxel<int> *> map_voxels;
        voxel_map_.get_all_pts(map_voxels);

        size_t reserve_num = 0;
        for (auto *vox : map_voxels) {
            if (vox == nullptr) {
                continue;
            }
            reserve_num += static_cast<size_t>(vox->point_seed_.size());
        }
        map_cloud->reserve(reserve_num);

        pcl::PointXYZI pt;
        pt.intensity = 1.0f;
        for (auto *vox : map_voxels) {
            if (vox == nullptr) {
                continue;
            }

            auto &seed = vox->point_seed_;
            for (int idx = 0; idx < seed.size(); idx++) {
                pt.x = seed.heap_[idx].x_;
                pt.y = seed.heap_[idx].y_;
                pt.z = seed.heap_[idx].z_;
                map_cloud->push_back(pt);
            }
        }
    }

    bool get_param_flg_use_smooth()
    {
        return param_flg_use_smooth_;
    }

#ifdef GEN_RGB_MAP
    bool get_rgb_submap(double &timestamp, Eigen::Quaterniond &pose_quat, Eigen::Vector3d &pose_pos, typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rgb_submap)
    {
        if (!param_enable_colorize_) {
            return false;
        }
        return rbg_keyframe_gen_.get_rgb_submap(timestamp, pose_quat, pose_pos, rgb_submap);
    }

    bool get_overlay_image(double &timestamp, cv::Mat &overlay_img)
    {
        if (!param_enable_colorize_) {
            return false;
        }
        return rbg_keyframe_gen_.get_overlay_image(timestamp, overlay_img);
    }
#endif

public:
    Sensor_Synchronizer<PointType> sensor_synchronizer_; //传感器同步类的实例

    RC_Vox_Map<RC_Vox, int, Voxel<int>> voxel_map_; // RC体素地图类的实例

#if MY_VIS_MODE
    typename pcl::PointCloud<PointType>::Ptr map_cloud_tmp_;
#endif

#if MY_DEBUG_MODE
    typename pcl::PointCloud<PointType>::Ptr predict_undistorted_cloud_;
    typename pcl::PointCloud<PointType>::Ptr predict_distorted_cloud_;
#endif

private:
    int subframe_num_; //子帧数
    bool flg_need_init_ = true; // 是否需要进行系统初始化
    unsigned long long frame_id_count_ = 0; // 当前处理的帧的id计数器
    size_t curr_frame_front_idx_; // 当前处理的帧在subframes_中的索引
    std::shared_ptr<Lidar_Imu_Measure_Group<PointType>> lidar_image_imu_syn_meas_; // 当前正在进行处理的激光-图像-IMU同步测量数据
    std::vector<Associated_Pair<PointType>> associated_pairs_; // 需要在初始化时resize
    std::deque<std::shared_ptr<Sub_Frame<PointType>>> subframes_;
    std::vector<std::shared_ptr<Sub_Frame<PointType>>> subframes_waiting_to_add_to_map_;

    Extrinsic_Group ext_group_; // 外参文件

    // （参数）配置文件位置
    std::string sensor_config_file_; // 构造函数中读入
    std::string system_dir_; // 构造函数中读入(文件夹须以/结尾)

    // （参数）系统初始化
    int param_initialization_method_ = 0; // 0:静态初始化， 1: 动态初始化， 2:外部初始化
    double param_static_initialize_time_;
    bool param_flg_estimate_imu_noise_;

    // （参数）激光雷达参数
    float param_detect_range_; // 点云数据关联时，认为匹配对儿有效的最大距离
    float param_detect_blind_; // 点云近距离盲区过滤阈值

#ifdef GEN_RGB_MAP
    bool param_enable_colorize_ = false;
    double param_td_cam_relative_imu_ = 0.0;
#endif

    // （参数）点云细分与平滑
    bool param_flg_auto_imu_noise_analysis_; // IMU噪声自动分析标志位
    bool param_flg_auto_subframe_num_analysis_ = true; // 是否根据系统信息自动判断细分帧数量
    int param_min_subframe_num_ = 20; // 非自动细分判断时的固定细分帧数量；自动细分判断时的最小细分帧数量
    int param_max_subframe_num_ = 20; // 非自动细分判断时的固定细分帧数量；自动细分判断时的最大细分帧数量
    double param_max_imu_acc_noise_inc_;
    double param_max_imu_gyr_noise_inc_;
    double param_high_dynamic_ratio_thres_ = 0.45; // 高动态触发阈值（运动强度比值）
    double param_high_dynamic_q_scale_ = 0.8; // 高动态下 IMU 过程噪声额外放大系数
    bool param_flg_residual_gating_ = true; // 是否启用点面残差门控
    double param_residual_gate_base_ = 0.6; // 点面残差门控基准阈值（m）
    double param_residual_gate_range_scale_ = 0.03; // 门控阈值随点距增长系数
    double param_residual_gate_dynamic_gain_ = 1.5; // 高动态下门控阈值放大增益
    bool param_flg_use_smooth_; // 是否开启平滑功能
    int param_smooth_iteration_num_; // 开启平滑功能时，每个细分帧平滑次数
    double param_thres_degrade_pos_; // 细分帧合入地图条件：平移自由度上不确定度阈值（std）
    double param_thres_degrade_ang_; // 细分帧合入地图条件：旋转自由度上不确定度阈值（std）
    // （参数）点云配准
    int param_min_point_num_for_registration_ = 50; // 子帧参与配准的最小点数，同时也是有效残差点数阈值
    int param_min_point_num_for_downsample_ = 10; // 子帧点数超过该值才降采样，且降采样后点数不低于该值才采用
    int param_max_iteration_num_max_ = 3; // 单子帧配准迭代的硬上限（for 循环上界）
    int param_max_rematch_num_max_ = 3; // 自动调参时 rematch 次数上限（用于计算当前帧 param_max_rematch_num_）
    int param_max_iteration_num_min_ = 3; // 自动调参时迭代次数下限（用于计算当前帧 param_max_iteration_num_）
    int param_max_rematch_num_min_ = 3; // 自动调参时 rematch 次数下限（用于计算当前帧 param_max_rematch_num_）
    int param_max_iteration_num_ = 3; // 当前帧生效的目标迭代次数（由 min/max 和运动强度 ratio 计算）
    int param_max_rematch_num_ = 3; // 当前帧生效的 rematch 次数阈值（超过后触发停止条件之一）
    int param_max_pt_searched_num_ = 5; // 预留：邻域搜索点数上限
    int param_min_pt_searched_num_ = 3; // 拟合局部平面的最少近邻点数（不足则该匹配点无效）
    double param_angle_converage_thres_; // IEKF 姿态增量收敛阈值
    double param_position_converage_thres_; // IEKF 位置增量收敛阈值
    double param_pos_error_thres_; // 预留：平移误差阈值
    double param_ang_error_thres_; // 预留：旋转误差阈值

    // （参数）其他
    double param_gravity_value_ = 9.81; // 重力加速度模长（m/s^2），用于系统初始化时构造重力向量
    int param_point_filter_num_ = 1; // 原始点云均匀抽样步长（每 N 点取 1 点，1 表示不过滤）
    double param_lidar_points_downsample_resolution_ = 0.2; // 体素降采样分辨率（leaf size，单位 m）
    bool param_flg_downsample_lidar_points_before_split_; // 配置开关：是否在点云细分前先做整帧降采样

    bool flg_downsample_lidar_points_before_split_; // 当前帧实际采用的前置降采样标志（会受子帧数规则影响）
    double motion_intensity_ratio_ = 0.0; // 当前帧运动强度比值（[0,1]）
    bool high_dynamic_mode_ = false; // 当前帧是否处于高动态模式
    double residual_gate_scale_ = 1.0; // 当前帧残差门控缩放系数

    // 保存哪些量 ，轨迹（高低频），耗时，细分帧数，使用点云数量，优化迭代次数，imu噪声。
    std::vector<int> subframe_nums_; // 每个激光帧自适应得到的细分帧数量记录
    std::vector<double> imu_acc_noise_stds_; // 每个激光帧生效的 IMU 加速度噪声标准差记录
    std::vector<double> imu_gyr_noise_stds_; // 每个激光帧生效的 IMU 角速度噪声标准差记录
    std::vector<int> v_pts_used_; // 每个激光帧用于后续处理的点数统计（细分后累计）
    std::vector<int> v_opt_iter_counts_; // 每个已处理细分帧的配准迭代次数记录
    std::vector<int> v_rematch_num_; // 每个已处理细分帧的 rematch 次数记录
    std::vector<State_Group> high_freq_traj_; // 高频轨迹：保存细分帧中的高频状态序列
    std::vector<State_Group> low_freq_traj_; // 低频轨迹：每个细分帧末状态（以及初始化状态）

#ifdef MY_DEBUG_MODE
    std::vector<double> max_accs_;
    std::vector<double> max_gyrs_;
    std::vector<double> max_vels_;
    std::vector<double> max_acc_stds_;
    std::vector<double> max_gyr_stds_;
    std::vector<double> max_vel_stds_;
    // std::vector<int> subframe_nums_;

    std::vector<Frame_Info> subframe_info_predict_states_;
    std::vector<Frame_Info> subframe_info_update_states_;
    std::vector<Frame_Info> frame_infos_;
    std::vector<int> debug_info_map_point_nums_;

    std::vector<double> v_dx_pos_;
    std::vector<double> v_dx_ang_;
    std::vector<double> v_dx_vel_;
#endif

#ifdef GEN_RGB_MAP
    Camera_Intrinsic cam_intrinsic_;
    Rgb_KeyFrame_Generation<PointType> rbg_keyframe_gen_;
    std::shared_ptr<Image_Data> latest_image_data_;
#endif
};