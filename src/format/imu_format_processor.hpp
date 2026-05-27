#pragma once
#include <iostream>
#include <string>
#include <Eigen/Core>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

/**
 * @brief IMU数据格式处理模块。
 *
 * 当前主要负责读入比例系数并对加速度进行统一缩放，
 * 便于不同数据源在同一物理量尺度下进入后端算法。
 */
class Imu_Format_Processor {
public:
    Imu_Format_Processor() = default;
    void load_config_parameter(const std::string &sensor_config_file)
    {
        YAML::Node config_node = YAML::LoadFile(sensor_config_file);

        try {
            param_acc_value_scale_ = config_node["param_acc_value_scale"].as<double>();
        } catch (...) {
            LOG(FATAL) << "[Imu_Format_Processor]: file " << sensor_config_file << " has a bad conversion";
        }

        LOG(INFO) << std::endl
                  << "| --------------- [Imu_Format_Processor parameter] --------------- |" << std::endl
                  << "[param_acc_value_scale_] " << param_acc_value_scale_ << std::endl
                  << std::endl;
    }

    void process(Eigen::Vector3d &acc_value)
    {
        acc_value *= param_acc_value_scale_;
    }

private:
    double param_acc_value_scale_;
};