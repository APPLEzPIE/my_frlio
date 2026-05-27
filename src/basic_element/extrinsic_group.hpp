#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "yaml-cpp/yaml.h"

/**
 * @brief 多传感器外参管理器。
 *
 * 统一维护 LiDAR-IMU、Camera-IMU、LiDAR-Camera 三组外参，
 * 支持从配置文件读取并提供各组合之间的查询接口。
 */
// 根据激光雷达与IMU以及相机与IMU间的外参，可以自动地在设置两个中任意一个外参时推断出激光雷达与相机之间的外参。
// TODO: 目前暂未实现三种传感器外参间的自动检测与求解。外部配置文件读入目前只对lio做了适配
struct Extrinsic_Group {
    /**
     *
     * @description:
     * @param {string&} sensor_config_file 配置文件
     * @return {*}
     * TODO: 目前需要提供 lidar与imu，以及lidar与camera间的外参，否则会报错。
     */
    void load_config_parameter(const std::string &sensor_config_file)
    {
        bool flg_success_between_l_i = load_extrinsic_between_lidar_imu(sensor_config_file);
        bool flg_success_between_c_i = load_extrinsic_between_camera_imu(sensor_config_file);
        bool flg_success_between_l_c = load_extrinsic_between_lidar_camera(sensor_config_file);

        std::cout << "flg_has_extrinsic_bewteen_ci_:" << flg_has_extrinsic_bewteen_ci_ << std::endl
                  << "flg_has_extrinsic_bewteen_li_:" << flg_has_extrinsic_bewteen_li_ << std::endl
                  << "flg_has_extrinsic_bewteen_cl_:" << flg_has_extrinsic_bewteen_cl_ << std::endl;

        // if(!flg_has_extrinsic_bewteen_ci_ || !flg_has_extrinsic_bewteen_li_ || !flg_has_extrinsic_bewteen_cl_) {
        //     LOG(FATAL) << "[Extrinsic_Group]: load extrinsic failure";
        // }
    }

    /**
     * @description: 根据配置文件读取lidar与image间的外参。需要参数文件中有
     *                  flg_extrinsic_bwtween_l_c_use_rot: 旋转部分是否使用旋转矩阵（不是的话就是使用四元数）
     *                  下面两组二选一
     *                  extrinsic_rot_l_c 与 extrinsic_quat_l_c（顺序x y z w ）二选一
     *                  extrinsic_trans_l_c
     *                  或者
     *                  extrinsic_rot_c_l 与 extrinsic_quat_c_l（顺序x y z w ）二选一
     *                  extrinsic_trans_c_l
     * @param {string&} sensor_config_file 配置文件
     * @return {bool} 是否成功得到外参
     */
    bool load_extrinsic_between_lidar_camera(const std::string &sensor_config_file)
    {
        YAML::Node config_node = YAML::LoadFile(sensor_config_file);
        bool flg_has_extrinsic_between_lc = true;

        std::vector<double> ext_rot_vec{ 9, 0.0 };
        std::vector<double> ext_quat_vec{ 4, 0.0 };
        std::vector<double> ext_trans_vec{ 3, 0.0 };
        Eigen::Matrix3d ext_rot;
        Eigen::Quaterniond ext_quat;
        Eigen::Vector3d ext_trans;
        try {
            bool flg_extrinsic_bwtween_li_use_rot = config_node["flg_extrinsic_bwtween_l_c_use_rot"].as<bool>();
            if (flg_extrinsic_bwtween_li_use_rot) {
                ext_rot_vec = config_node["extrinsic_rot_l_c"].as<std::vector<double>>();
                ext_rot << ext_rot_vec[0], ext_rot_vec[1], ext_rot_vec[2], ext_rot_vec[3], ext_rot_vec[4], ext_rot_vec[5], ext_rot_vec[6], ext_rot_vec[7], ext_rot_vec[8];
            } else {
                ext_quat_vec = config_node["extrinsic_quat_l_c"].as<std::vector<double>>(); // x y z w
                ext_quat = Eigen::Quaterniond(ext_quat_vec[3], ext_quat_vec[0], ext_quat_vec[1], ext_quat_vec[2]); // w x y z
            }
            ext_trans_vec = config_node["extrinsic_trans_l_c"].as<std::vector<double>>();
            ext_trans << ext_trans_vec[0], ext_trans_vec[1], ext_trans_vec[2];

            if (flg_extrinsic_bwtween_li_use_rot) {
                set_extrinsic_l_c(ext_rot, ext_trans);
            } else {
                set_extrinsic_l_c(ext_quat, ext_trans);
            }
        } catch (...) {
            flg_has_extrinsic_between_lc = false;
        }

        if (flg_has_extrinsic_between_lc == false) {
            flg_has_extrinsic_between_lc = true;
            try {
                bool flg_extrinsic_bwtween_lc_use_rot = config_node["flg_extrinsic_bwtween_l_c_use_rot"].as<bool>();
                if (flg_extrinsic_bwtween_lc_use_rot) {
                    ext_rot_vec = config_node["extrinsic_rot_c_l"].as<std::vector<double>>();
                    ext_rot << ext_rot_vec[0], ext_rot_vec[1], ext_rot_vec[2], ext_rot_vec[3], ext_rot_vec[4], ext_rot_vec[5], ext_rot_vec[6], ext_rot_vec[7], ext_rot_vec[8];
                } else {
                    ext_quat_vec = config_node["extrinsic_quat_c_l"].as<std::vector<double>>(); // x y z w
                    ext_quat = Eigen::Quaterniond(ext_quat_vec[3], ext_quat_vec[0], ext_quat_vec[1], ext_quat_vec[2]); // w x y z
                }
                ext_trans_vec = config_node["extrinsic_trans_c_l"].as<std::vector<double>>();
                ext_trans << ext_trans_vec[0], ext_trans_vec[1], ext_trans_vec[2];

                if (flg_extrinsic_bwtween_lc_use_rot) {
                    set_extrinsic_c_l(ext_rot, ext_trans);
                } else {
                    set_extrinsic_c_l(ext_quat, ext_trans);
                }
            } catch (...) {
                flg_has_extrinsic_between_lc = false;
            }
        }

        if (!flg_has_extrinsic_between_lc) {
            LOG(ERROR) << "[Extrinsic_Group]: file " << sensor_config_file << " has no extrinsic bewteen lidar and camera";
            return false;
        } else {
            return true;
        }
    }

    /**
     * @description: 根据配置文件读取lidar与imu间的外参。需要参数文件中有
     *                  flg_extrinsic_bwtween_l_i_use_rot: 旋转部分是否使用旋转矩阵（不是的话就是使用四元数）
     *                  下面两组二选一
     *                  extrinsic_rot_l_i 与 extrinsic_quat_l_i（顺序x y z w ）二选一
     *                  extrinsic_trans_l_i
     *                  或者
     *                  extrinsic_rot_i_l 与 extrinsic_quat_i_l（顺序x y z w ）二选一
     *                  extrinsic_trans_i_l
     * @param {string&} sensor_config_file 配置文件
     * @return {bool} 是否成功得到外参
     */
    bool load_extrinsic_between_lidar_imu(const std::string &sensor_config_file)
    {
        YAML::Node config_node = YAML::LoadFile(sensor_config_file);

        bool flg_has_extrinsic_between_l_i = true;
        {
            std::vector<double> ext_rot_vec{ 9, 0.0 };
            std::vector<double> ext_quat_vec{ 4, 0.0 };
            std::vector<double> ext_trans_vec{ 3, 0.0 };
            Eigen::Matrix3d ext_rot;
            Eigen::Quaterniond ext_quat;
            Eigen::Vector3d ext_trans;
            try {
                bool flg_extrinsic_bwtween_l_i_use_rot = config_node["flg_extrinsic_bwtween_l_i_use_rot"].as<bool>();
                if (flg_extrinsic_bwtween_l_i_use_rot) {
                    ext_rot_vec = config_node["extrinsic_rot_l_i"].as<std::vector<double>>();
                    ext_rot << ext_rot_vec[0], ext_rot_vec[1], ext_rot_vec[2], ext_rot_vec[3], ext_rot_vec[4], ext_rot_vec[5], ext_rot_vec[6], ext_rot_vec[7], ext_rot_vec[8];
                } else {
                    ext_quat_vec = config_node["extrinsic_quat_l_i"].as<std::vector<double>>(); // x y z w
                    ext_quat = Eigen::Quaterniond(ext_quat_vec[3], ext_quat_vec[0], ext_quat_vec[1], ext_quat_vec[2]); // w x y z
                }
                ext_trans_vec = config_node["extrinsic_trans_l_i"].as<std::vector<double>>();
                ext_trans << ext_trans_vec[0], ext_trans_vec[1], ext_trans_vec[2];

                if (flg_extrinsic_bwtween_l_i_use_rot) {
                    set_extrinsic_l_i(ext_rot, ext_trans);
                } else {
                    set_extrinsic_l_i(ext_quat, ext_trans);
                }
            } catch (...) {
                flg_has_extrinsic_between_l_i = false;
            }

            if (flg_has_extrinsic_between_l_i == false) {
                flg_has_extrinsic_between_l_i = true;
                try {
                    bool flg_extrinsic_bwtween_l_i_use_rot = config_node["flg_extrinsic_bwtween_l_i_use_rot"].as<bool>();
                    if (flg_extrinsic_bwtween_l_i_use_rot) {
                        ext_rot_vec = config_node["extrinsic_rot_i_l"].as<std::vector<double>>();
                        ext_rot << ext_rot_vec[0], ext_rot_vec[1], ext_rot_vec[2], ext_rot_vec[3], ext_rot_vec[4], ext_rot_vec[5], ext_rot_vec[6], ext_rot_vec[7], ext_rot_vec[8];
                    } else {
                        ext_quat_vec = config_node["extrinsic_quat_i_l"].as<std::vector<double>>(); // x y z w
                        ext_quat = Eigen::Quaterniond(ext_quat_vec[3], ext_quat_vec[0], ext_quat_vec[1], ext_quat_vec[2]); // w x y z
                    }
                    ext_trans_vec = config_node["extrinsic_trans_i_l"].as<std::vector<double>>();
                    ext_trans << ext_trans_vec[0], ext_trans_vec[1], ext_trans_vec[2];

                    if (flg_extrinsic_bwtween_l_i_use_rot) {
                        set_extrinsic_i_l(ext_rot, ext_trans);
                    } else {
                        set_extrinsic_i_l(ext_quat, ext_trans);
                    }
                } catch (...) {
                    flg_has_extrinsic_between_l_i = false;
                }
            }
        }

        if (!flg_has_extrinsic_between_l_i) {
            LOG(ERROR) << "[Extrinsic_Group]: file " << sensor_config_file << " has no extrinsic bewteen lidar and imu";
            return false;
        } else {
            return true;
        }
    }

    /**
     * @description: 根据配置文件读取lidar与camera间的外参。需要参数文件中有
     *                  flg_extrinsic_bwtween_l_i_use_rot: 旋转部分是否使用旋转矩阵（不是的话就是使用四元数）
     *                  下面两组二选一
     *                  extrinsic_rot_l_i 与 extrinsic_quat_l_i（顺序x y z w ）二选一
     *                  extrinsic_trans_l_i
     *                  或者
     *                  extrinsic_rot_i_l 与 extrinsic_quat_i_l（顺序x y z w ）二选一
     *                  extrinsic_trans_i_l
     * @param {string&} sensor_config_file 配置文件
     * @return {*} 是否成功得到外参
     */
    bool load_extrinsic_between_camera_imu(const std::string &sensor_config_file)
    {
        YAML::Node config_node = YAML::LoadFile(sensor_config_file);

        bool flg_has_extrinsic_between_c_i = true;
        {
            std::vector<double> ext_rot_vec{ 9, 0.0 };
            std::vector<double> ext_quat_vec{ 4, 0.0 };
            std::vector<double> ext_trans_vec{ 3, 0.0 };
            Eigen::Matrix3d ext_rot;
            Eigen::Quaterniond ext_quat;
            Eigen::Vector3d ext_trans;
            try {
                bool flg_extrinsic_bwtween_c_i_use_rot = config_node["flg_extrinsic_bwtween_c_i_use_rot"].as<bool>();
                if (flg_extrinsic_bwtween_c_i_use_rot) {
                    ext_rot_vec = config_node["extrinsic_rot_c_i"].as<std::vector<double>>();
                    ext_rot << ext_rot_vec[0], ext_rot_vec[1], ext_rot_vec[2], ext_rot_vec[3], ext_rot_vec[4], ext_rot_vec[5], ext_rot_vec[6], ext_rot_vec[7], ext_rot_vec[8];
                } else {
                    ext_quat_vec = config_node["extrinsic_quat_c_i"].as<std::vector<double>>(); // x y z w
                    ext_quat = Eigen::Quaterniond(ext_quat_vec[3], ext_quat_vec[0], ext_quat_vec[1], ext_quat_vec[2]); // w x y z
                }
                ext_trans_vec = config_node["extrinsic_trans_c_i"].as<std::vector<double>>();
                ext_trans << ext_trans_vec[0], ext_trans_vec[1], ext_trans_vec[2];

                if (flg_extrinsic_bwtween_c_i_use_rot) {
                    set_extrinsic_c_i(ext_rot, ext_trans);
                } else {
                    set_extrinsic_c_i(ext_quat, ext_trans);
                }
            } catch (...) {
                flg_has_extrinsic_between_c_i = false;
            }

            if (flg_has_extrinsic_between_c_i == false) {
                flg_has_extrinsic_between_c_i = true;
                try {
                    bool flg_extrinsic_bwtween_c_i_use_rot = config_node["flg_extrinsic_bwtween_c_i_use_rot"].as<bool>();
                    if (flg_extrinsic_bwtween_c_i_use_rot) {
                        ext_rot_vec = config_node["extrinsic_rot_i_c"].as<std::vector<double>>();
                        ext_rot << ext_rot_vec[0], ext_rot_vec[1], ext_rot_vec[2], ext_rot_vec[3], ext_rot_vec[4], ext_rot_vec[5], ext_rot_vec[6], ext_rot_vec[7], ext_rot_vec[8];
                    } else {
                        ext_quat_vec = config_node["extrinsic_quat_i_c"].as<std::vector<double>>(); // x y z w
                        ext_quat = Eigen::Quaterniond(ext_quat_vec[3], ext_quat_vec[0], ext_quat_vec[1], ext_quat_vec[2]); // w x y z
                    }
                    ext_trans_vec = config_node["extrinsic_trans_i_c"].as<std::vector<double>>();
                    ext_trans << ext_trans_vec[0], ext_trans_vec[1], ext_trans_vec[2];

                    if (flg_extrinsic_bwtween_c_i_use_rot) {
                        set_extrinsic_i_c(ext_rot, ext_trans);
                    } else {
                        set_extrinsic_i_c(ext_quat, ext_trans);
                    }
                } catch (...) {
                    flg_has_extrinsic_between_c_i = false;
                }
            }
        }

        if (!flg_has_extrinsic_between_c_i) {
            LOG(ERROR) << "[Extrinsic_Group]: file " << sensor_config_file << " has no extrinsic bewteen camera and imu";
            return false;
        } else {
            return true;
        }
    }

    // 弃用
    // void load_config_parameter(const std::string& sensor_config_file) {
    //     YAML::Node config_node = YAML::LoadFile(sensor_config_file);

    //     bool flg_success_between_l_i = ;
    //     bool flg_success_between_c_i = ;
    //     bool flg_success_between_l_c = ;

    //     bool flg_has_extrinsic_between_l_i = true;
    //     {
    //         std::vector<double> ext_rot_vec{9, 0.0};
    //         std::vector<double> ext_quat_vec{4, 0.0};
    //         std::vector<double> ext_trans_vec{3, 0.0};
    //         Eigen::Matrix3d ext_rot;
    //         Eigen::Quaterniond ext_quat;
    //         Eigen::Vector3d ext_trans;
    //         try {
    //             bool flg_extrinsic_bwtween_li_use_rot = config_node["flg_extrinsic_bwtween_l_i_use_rot"].as<bool>();
    //             if(flg_extrinsic_bwtween_li_use_rot) {
    //                 ext_rot_vec = config_node["extrinsic_rot_l_i"].as<std::vector<double>>();
    //                 ext_rot <<  ext_rot_vec[0], ext_rot_vec[1], ext_rot_vec[2],
    //                             ext_rot_vec[3], ext_rot_vec[4], ext_rot_vec[5],
    //                             ext_rot_vec[6], ext_rot_vec[7], ext_rot_vec[8];
    //             } else {
    //                 ext_quat_vec = config_node["extrinsic_quat_l_i"].as<std::vector<double>>(); // x y z w
    //                 ext_quat = Eigen::Quaterniond(ext_quat_vec[3], ext_quat_vec[0], ext_quat_vec[1], ext_quat_vec[2]); // w x y z
    //             }
    //             ext_trans_vec = config_node["extrinsic_trans_l_i"].as<std::vector<double>>();
    //             ext_trans << ext_trans_vec[0], ext_trans_vec[1], ext_trans_vec[2];

    //             if(flg_extrinsic_bwtween_li_use_rot) {
    //                 set_extrinsic_l_i(ext_rot, ext_trans);
    //             } else {
    //                 set_extrinsic_l_i(ext_quat, ext_trans);
    //             }
    //         } catch (...) {
    //             flg_has_extrinsic_between_l_i = false;
    //         }

    //         if(flg_has_extrinsic_between_l_i == false) {
    //             flg_has_extrinsic_between_l_i = true;
    //             try {
    //                 bool flg_extrinsic_bwtween_li_use_rot = config_node["flg_extrinsic_bwtween_l_i_use_rot"].as<bool>();
    //                 if(flg_extrinsic_bwtween_li_use_rot) {
    //                     ext_rot_vec = config_node["extrinsic_rot_i_l"].as<std::vector<double>>();
    //                     ext_rot <<  ext_rot_vec[0], ext_rot_vec[1], ext_rot_vec[2],
    //                                 ext_rot_vec[3], ext_rot_vec[4], ext_rot_vec[5],
    //                                 ext_rot_vec[6], ext_rot_vec[7], ext_rot_vec[8];
    //                 } else {
    //                     ext_quat_vec = config_node["extrinsic_quat_i_l"].as<std::vector<double>>(); // x y z w
    //                     ext_quat = Eigen::Quaterniond(ext_quat_vec[3], ext_quat_vec[0], ext_quat_vec[1], ext_quat_vec[2]); // w x y z
    //                 }
    //                 ext_trans_vec = config_node["extrinsic_trans_i_l"].as<std::vector<double>>();
    //                 ext_trans << ext_trans_vec[0], ext_trans_vec[1], ext_trans_vec[2];

    //                 if(flg_extrinsic_bwtween_li_use_rot) {
    //                     set_extrinsic_i_l(ext_rot, ext_trans);
    //                 } else {
    //                     set_extrinsic_i_l(ext_quat, ext_trans);
    //                 }
    //             } catch (...) {
    //                 flg_has_extrinsic_between_l_i = false;
    //             }
    //         }
    //     }

    //     if(!flg_has_extrinsic_between_l_i) {
    //         LOG(FATAL) << "[Extrinsic_Group]: file " << sensor_config_file <<" has no extrinsic bewteen lidar and imu";
    //     }
    // }

    void set_extrinsic_l_i(const Eigen::Matrix3d &extrinsic_rot_l_i, const Eigen::Vector3d &extrinsic_trans_l_i)
    {
        flg_has_extrinsic_bewteen_li_ = true;
        quat_l_i_ = Eigen::Quaterniond(extrinsic_rot_l_i);
        rot_l_i_ = extrinsic_rot_l_i;
        trans_l_i_ = extrinsic_trans_l_i;
        quat_i_l_ = quat_l_i_.inverse();
        rot_i_l_ = quat_i_l_.toRotationMatrix();
        trans_i_l_ = quat_i_l_ * (-trans_l_i_);

        if (!flg_has_extrinsic_bewteen_cl_ && flg_has_extrinsic_bewteen_ci_) {
            flg_has_extrinsic_bewteen_cl_ = true;
            quat_c_l_ = quat_c_i_ * quat_i_l_;
            rot_c_l_ = quat_c_l_.toRotationMatrix();
            trans_c_l_ = quat_c_i_ * trans_i_l_ + trans_c_i_;
            quat_l_c_ = quat_c_l_.inverse();
            rot_l_c_ = quat_l_c_.toRotationMatrix();
            trans_l_c_ = quat_l_c_ * (-trans_c_l_);
        }

        if (!flg_has_extrinsic_bewteen_ci_ && flg_has_extrinsic_bewteen_cl_) {
            flg_has_extrinsic_bewteen_ci_ = true;
            quat_c_i_ = quat_c_l_ * quat_l_i_;
            trans_c_i_ = quat_c_l_ * trans_l_i_ + trans_c_l_;
            quat_i_c_ = quat_c_i_.inverse();
            trans_i_c_ = quat_i_c_ * (-trans_c_i_);

            rot_c_i_ = quat_c_i_.toRotationMatrix();
            rot_i_c_ = quat_i_c_.toRotationMatrix();
        }
    }

    void set_extrinsic_l_i(const Eigen::Quaterniond &extrinsic_quat_l_i, const Eigen::Vector3d &extrinsic_trans_l_i)
    {
        flg_has_extrinsic_bewteen_li_ = true;
        quat_l_i_ = extrinsic_quat_l_i;
        rot_l_i_ = quat_l_i_.toRotationMatrix();
        trans_l_i_ = extrinsic_trans_l_i;
        quat_i_l_ = quat_l_i_.inverse();
        rot_i_l_ = quat_i_l_.toRotationMatrix();
        trans_i_l_ = quat_i_l_ * (-trans_l_i_);

        if (!flg_has_extrinsic_bewteen_cl_ && flg_has_extrinsic_bewteen_ci_) {
            flg_has_extrinsic_bewteen_cl_ = true;
            quat_c_l_ = quat_c_i_ * quat_i_l_;
            rot_c_l_ = quat_c_l_.toRotationMatrix();
            trans_c_l_ = quat_c_i_ * trans_i_l_ + trans_c_i_;
            quat_l_c_ = quat_c_l_.inverse();
            rot_l_c_ = quat_l_c_.toRotationMatrix();
            trans_l_c_ = quat_l_c_ * (-trans_c_l_);
        }

        if (!flg_has_extrinsic_bewteen_ci_ && flg_has_extrinsic_bewteen_cl_) {
            flg_has_extrinsic_bewteen_ci_ = true;
            quat_c_i_ = quat_c_l_ * quat_l_i_;
            trans_c_i_ = quat_c_l_ * trans_l_i_ + trans_c_l_;
            quat_i_c_ = quat_c_i_.inverse();
            trans_i_c_ = quat_i_c_ * (-trans_c_i_);

            rot_c_i_ = quat_c_i_.toRotationMatrix();
            rot_i_c_ = quat_i_c_.toRotationMatrix();
        }
    }

    void set_extrinsic_i_l(const Eigen::Matrix3d &extrinsic_rot_i_l, const Eigen::Vector3d &extrinsic_trans_i_l)
    {
        flg_has_extrinsic_bewteen_li_ = true;
        quat_i_l_ = Eigen::Quaterniond(extrinsic_rot_i_l);
        rot_i_l_ = extrinsic_rot_i_l;
        trans_i_l_ = extrinsic_trans_i_l;
        quat_l_i_ = quat_i_l_.inverse();
        rot_l_i_ = quat_l_i_.toRotationMatrix();
        trans_l_i_ = quat_l_i_ * (-trans_i_l_);

        if (!flg_has_extrinsic_bewteen_cl_ && flg_has_extrinsic_bewteen_ci_) {
            flg_has_extrinsic_bewteen_cl_ = true;
            quat_c_l_ = quat_c_i_ * quat_i_l_;
            rot_c_l_ = quat_c_l_.toRotationMatrix();
            trans_c_l_ = quat_c_i_ * trans_i_l_ + trans_c_i_;
            quat_l_c_ = quat_c_l_.inverse();
            rot_l_c_ = quat_l_c_.toRotationMatrix();
            trans_l_c_ = quat_l_c_ * (-trans_c_l_);
        }

        if (!flg_has_extrinsic_bewteen_ci_ && flg_has_extrinsic_bewteen_cl_) {
            flg_has_extrinsic_bewteen_ci_ = true;
            quat_c_i_ = quat_c_l_ * quat_l_i_;
            trans_c_i_ = quat_c_l_ * trans_l_i_ + trans_c_l_;
            quat_i_c_ = quat_c_i_.inverse();
            trans_i_c_ = quat_i_c_ * (-trans_c_i_);

            rot_c_i_ = quat_c_i_.toRotationMatrix();
            rot_i_c_ = quat_i_c_.toRotationMatrix();
        }
    }

    void set_extrinsic_i_l(const Eigen::Quaterniond &extrinsic_quat_i_l, const Eigen::Vector3d &extrinsic_trans_i_l)
    {
        flg_has_extrinsic_bewteen_li_ = true;
        quat_i_l_ = extrinsic_quat_i_l;
        rot_i_l_ = quat_i_l_.toRotationMatrix();
        trans_i_l_ = extrinsic_trans_i_l;
        quat_l_i_ = quat_i_l_.inverse();
        rot_l_i_ = quat_l_i_.toRotationMatrix();
        trans_l_i_ = quat_l_i_ * (-trans_i_l_);

        if (!flg_has_extrinsic_bewteen_cl_ && flg_has_extrinsic_bewteen_ci_) {
            flg_has_extrinsic_bewteen_cl_ = true;
            quat_c_l_ = quat_c_i_ * quat_i_l_;
            rot_c_l_ = quat_c_l_.toRotationMatrix();
            trans_c_l_ = quat_c_i_ * trans_i_l_ + trans_c_i_;
            quat_l_c_ = quat_c_l_.inverse();
            rot_l_c_ = quat_l_c_.toRotationMatrix();
            trans_l_c_ = quat_l_c_ * (-trans_c_l_);
        }

        if (!flg_has_extrinsic_bewteen_ci_ && flg_has_extrinsic_bewteen_cl_) {
            flg_has_extrinsic_bewteen_ci_ = true;
            quat_c_i_ = quat_c_l_ * quat_l_i_;
            trans_c_i_ = quat_c_l_ * trans_l_i_ + trans_c_l_;
            quat_i_c_ = quat_c_i_.inverse();
            trans_i_c_ = quat_i_c_ * (-trans_c_i_);

            rot_c_i_ = quat_c_i_.toRotationMatrix();
            rot_i_c_ = quat_i_c_.toRotationMatrix();
        }
    }

    void set_extrinsic_c_i(const Eigen::Quaterniond &extrinsic_q_c2i, const Eigen::Vector3d &extrinsic_t_c2i)
    {
        flg_has_extrinsic_bewteen_ci_ = true;
        quat_c_i_ = extrinsic_q_c2i;
        trans_c_i_ = extrinsic_t_c2i;
        quat_i_c_ = quat_c_i_.inverse();
        trans_i_c_ = quat_i_c_ * (-trans_c_i_);

        if (!flg_has_extrinsic_bewteen_cl_ && flg_has_extrinsic_bewteen_li_) {
            flg_has_extrinsic_bewteen_cl_ = true;
            quat_c_l_ = quat_c_i_ * quat_i_l_;
            trans_c_l_ = quat_c_i_ * trans_i_l_ + trans_c_i_;
            quat_l_c_ = quat_c_l_.inverse();
            trans_l_c_ = quat_l_c_ * (-trans_c_l_);

            rot_c_l_ = quat_c_l_.toRotationMatrix();
            rot_l_c_ = quat_l_c_.toRotationMatrix();
        }

        if (!flg_has_extrinsic_bewteen_li_ && flg_has_extrinsic_bewteen_cl_) {
            flg_has_extrinsic_bewteen_li_ = true;
            quat_l_i_ = quat_l_c_ * quat_c_i_;
            trans_l_i_ = quat_l_c_ * trans_c_i_ + trans_l_c_;
            quat_i_l_ = quat_l_i_.inverse();
            trans_i_l_ = quat_i_l_ * (-trans_l_i_);

            rot_l_i_ = quat_l_i_.toRotationMatrix();
            rot_i_l_ = quat_i_l_.toRotationMatrix();
        }
    }

    void set_extrinsic_i_c(const Eigen::Quaterniond &extrinsic_q_i2c, const Eigen::Vector3d &extrinsic_t_i2c)
    {
        flg_has_extrinsic_bewteen_ci_ = true;
        quat_i_c_ = extrinsic_q_i2c;
        trans_i_c_ = extrinsic_t_i2c;
        quat_c_i_ = quat_i_c_.inverse();
        trans_c_i_ = quat_c_i_ * (-trans_i_c_);

        if (!flg_has_extrinsic_bewteen_cl_ && flg_has_extrinsic_bewteen_li_) {
            flg_has_extrinsic_bewteen_cl_ = true;
            quat_c_l_ = quat_c_i_ * quat_i_l_;
            trans_c_l_ = quat_c_i_ * trans_i_l_ + trans_c_i_;
            quat_l_c_ = quat_c_l_.inverse();
            trans_l_c_ = quat_l_c_ * (-trans_c_l_);

            rot_c_l_ = quat_c_l_.toRotationMatrix();
            rot_l_c_ = quat_l_c_.toRotationMatrix();
        }

        if (!flg_has_extrinsic_bewteen_li_ && flg_has_extrinsic_bewteen_cl_) {
            flg_has_extrinsic_bewteen_li_ = true;
            quat_l_i_ = quat_l_c_ * quat_c_i_;
            trans_l_i_ = quat_l_c_ * trans_c_i_ + trans_l_c_;
            quat_i_l_ = quat_l_i_.inverse();
            trans_i_l_ = quat_i_l_ * (-trans_l_i_);

            rot_l_i_ = quat_l_i_.toRotationMatrix();
            rot_i_l_ = quat_i_l_.toRotationMatrix();
        }
    }

    void set_extrinsic_c_i(const Eigen::Matrix3d &extrinsic_r_c2i, const Eigen::Vector3d &extrinsic_t_c2i)
    {
        flg_has_extrinsic_bewteen_ci_ = true;
        quat_c_i_ = Eigen::Quaterniond(extrinsic_r_c2i);
        trans_c_i_ = extrinsic_t_c2i;
        quat_i_c_ = quat_c_i_.inverse();
        trans_i_c_ = quat_i_c_ * (-trans_c_i_);

        rot_i_c_ = quat_i_c_.toRotationMatrix();
        rot_c_i_ = extrinsic_r_c2i;

        if (!flg_has_extrinsic_bewteen_cl_ && flg_has_extrinsic_bewteen_li_) {
            flg_has_extrinsic_bewteen_cl_ = true;
            quat_c_l_ = quat_c_i_ * quat_i_l_;
            trans_c_l_ = quat_c_i_ * trans_i_l_ + trans_c_i_;
            quat_l_c_ = quat_c_l_.inverse();
            trans_l_c_ = quat_l_c_ * (-trans_c_l_);

            rot_c_l_ = quat_c_l_.toRotationMatrix();
            rot_l_c_ = quat_l_c_.toRotationMatrix();
        }

        if (!flg_has_extrinsic_bewteen_li_ && flg_has_extrinsic_bewteen_cl_) {
            flg_has_extrinsic_bewteen_li_ = true;
            quat_l_i_ = quat_l_c_ * quat_c_i_;
            trans_l_i_ = quat_l_c_ * trans_c_i_ + trans_l_c_;
            quat_i_l_ = quat_l_i_.inverse();
            trans_i_l_ = quat_i_l_ * (-trans_l_i_);

            rot_l_i_ = quat_l_i_.toRotationMatrix();
            rot_i_l_ = quat_i_l_.toRotationMatrix();
        }
    }

    void set_extrinsic_i_c(const Eigen::Matrix3d &extrinsic_r_i2c, const Eigen::Vector3d &extrinsic_t_i2c)
    {
        flg_has_extrinsic_bewteen_ci_ = true;
        quat_i_c_ = Eigen::Quaterniond(extrinsic_r_i2c);
        trans_i_c_ = extrinsic_t_i2c;
        quat_c_i_ = quat_i_c_.inverse();
        trans_c_i_ = quat_c_i_ * (-trans_i_c_);

        rot_i_c_ = extrinsic_r_i2c;
        rot_c_i_ = quat_c_i_.toRotationMatrix();

        if (!flg_has_extrinsic_bewteen_cl_ && flg_has_extrinsic_bewteen_li_) {
            flg_has_extrinsic_bewteen_cl_ = true;
            quat_c_l_ = quat_c_i_ * quat_i_l_;
            trans_c_l_ = quat_c_i_ * trans_i_l_ + trans_c_i_;
            quat_l_c_ = quat_c_l_.inverse();
            trans_l_c_ = quat_l_c_ * (-trans_c_l_);

            rot_c_l_ = quat_c_l_.toRotationMatrix();
            rot_l_c_ = quat_l_c_.toRotationMatrix();
        }

        if (!flg_has_extrinsic_bewteen_li_ && flg_has_extrinsic_bewteen_cl_) {
            flg_has_extrinsic_bewteen_li_ = true;
            quat_l_i_ = quat_l_c_ * quat_c_i_;
            trans_l_i_ = quat_l_c_ * trans_c_i_ + trans_l_c_;
            quat_i_l_ = quat_l_i_.inverse();
            trans_i_l_ = quat_i_l_ * (-trans_l_i_);

            rot_l_i_ = quat_l_i_.toRotationMatrix();
            rot_i_l_ = quat_i_l_.toRotationMatrix();
        }
    }

    void set_extrinsic_c_l(const Eigen::Quaterniond &extrinsic_quat_c_l, const Eigen::Vector3d &extrinsic_trans_c_l)
    {
        flg_has_extrinsic_bewteen_cl_ = true;
        quat_c_l_ = extrinsic_quat_c_l;
        trans_c_l_ = extrinsic_trans_c_l;
        quat_l_c_ = quat_c_l_.inverse();
        trans_l_c_ = quat_l_c_ * (-trans_c_l_);

        rot_c_l_ = quat_c_l_.toRotationMatrix();
        rot_l_c_ = quat_l_c_.toRotationMatrix();

        if (!flg_has_extrinsic_bewteen_ci_ && flg_has_extrinsic_bewteen_li_) {
            flg_has_extrinsic_bewteen_ci_ = true;
            quat_c_i_ = quat_c_l_ * quat_l_i_;
            trans_c_i_ = quat_c_l_ * trans_l_i_ + trans_c_l_;
            quat_i_c_ = quat_c_i_.inverse();
            trans_i_c_ = quat_i_c_ * (-trans_c_i_);

            rot_c_i_ = quat_c_i_.toRotationMatrix();
            rot_i_c_ = quat_i_c_.toRotationMatrix();
        }

        if (!flg_has_extrinsic_bewteen_li_ && flg_has_extrinsic_bewteen_ci_) {
            flg_has_extrinsic_bewteen_li_ = true;
            quat_l_i_ = quat_l_c_ * quat_c_i_;
            trans_l_i_ = quat_l_c_ * trans_c_i_ + trans_l_c_;
            quat_i_l_ = quat_l_i_.inverse();
            trans_i_l_ = quat_i_l_ * (-trans_l_i_);

            rot_l_i_ = quat_l_i_.toRotationMatrix();
            rot_i_l_ = quat_i_l_.toRotationMatrix();
        }
    }

    void set_extrinsic_l_c(const Eigen::Quaterniond &extrinsic_quat_l_c, const Eigen::Vector3d &extrinsic_trans_l_c)
    {
        flg_has_extrinsic_bewteen_cl_ = true;
        quat_l_c_ = extrinsic_quat_l_c;
        trans_l_c_ = extrinsic_trans_l_c;
        quat_c_l_ = quat_l_c_.inverse();
        trans_c_l_ = quat_c_l_ * (-trans_l_c_);

        rot_c_l_ = quat_c_l_.toRotationMatrix();
        rot_l_c_ = quat_l_c_.toRotationMatrix();

        if (!flg_has_extrinsic_bewteen_ci_ && flg_has_extrinsic_bewteen_li_) {
            flg_has_extrinsic_bewteen_ci_ = true;
            quat_c_i_ = quat_c_l_ * quat_l_i_;
            trans_c_i_ = quat_c_l_ * trans_l_i_ + trans_c_l_;
            quat_i_c_ = quat_c_i_.inverse();
            trans_i_c_ = quat_i_c_ * (-trans_c_i_);

            rot_c_i_ = quat_c_i_.toRotationMatrix();
            rot_i_c_ = quat_i_c_.toRotationMatrix();
        }

        if (!flg_has_extrinsic_bewteen_li_ && flg_has_extrinsic_bewteen_ci_) {
            flg_has_extrinsic_bewteen_li_ = true;
            quat_l_i_ = quat_l_c_ * quat_c_i_;
            trans_l_i_ = quat_l_c_ * trans_c_i_ + trans_l_c_;
            quat_i_l_ = quat_l_i_.inverse();
            trans_i_l_ = quat_i_l_ * (-trans_l_i_);

            rot_l_i_ = quat_l_i_.toRotationMatrix();
            rot_i_l_ = quat_i_l_.toRotationMatrix();
        }
    }

    void set_extrinsic_c_l(const Eigen::Matrix3d &extrinsic_rot_c_l, const Eigen::Vector3d &extrinsic_trans_c_l)
    {
        flg_has_extrinsic_bewteen_cl_ = true;
        quat_c_l_ = Eigen::Quaterniond(extrinsic_rot_c_l);
        trans_c_l_ = extrinsic_trans_c_l;
        quat_l_c_ = quat_c_l_.inverse();
        trans_l_c_ = quat_l_c_ * (-trans_c_l_);

        rot_c_l_ = extrinsic_rot_c_l;
        rot_l_c_ = quat_l_c_.toRotationMatrix();

        if (!flg_has_extrinsic_bewteen_ci_ && flg_has_extrinsic_bewteen_li_) {
            flg_has_extrinsic_bewteen_ci_ = true;
            quat_c_i_ = quat_c_l_ * quat_l_i_;
            trans_c_i_ = quat_c_l_ * trans_l_i_ + trans_c_l_;
            quat_i_c_ = quat_c_i_.inverse();
            trans_i_c_ = quat_i_c_ * (-trans_c_i_);

            rot_c_i_ = quat_c_i_.toRotationMatrix();
            rot_i_c_ = quat_i_c_.toRotationMatrix();
        }

        if (!flg_has_extrinsic_bewteen_li_ && flg_has_extrinsic_bewteen_ci_) {
            flg_has_extrinsic_bewteen_li_ = true;
            quat_l_i_ = quat_l_c_ * quat_c_i_;
            trans_l_i_ = quat_l_c_ * trans_c_i_ + trans_l_c_;
            quat_i_l_ = quat_l_i_.inverse();
            trans_i_l_ = quat_i_l_ * (-trans_l_i_);

            rot_l_i_ = quat_l_i_.toRotationMatrix();
            rot_i_l_ = quat_i_l_.toRotationMatrix();
        }
    }

    void set_extrinsic_l_c(const Eigen::Matrix3d &extrinsic_rot_l_c, const Eigen::Vector3d &extrinsic_trans_l_c)
    {
        flg_has_extrinsic_bewteen_cl_ = true;
        quat_l_c_ = Eigen::Quaterniond(extrinsic_rot_l_c);
        trans_l_c_ = extrinsic_trans_l_c;
        quat_c_l_ = quat_l_c_.inverse();
        trans_c_l_ = quat_c_l_ * (-trans_l_c_);

        rot_c_l_ = quat_c_l_.toRotationMatrix();
        rot_l_c_ = extrinsic_rot_l_c;

        if (!flg_has_extrinsic_bewteen_ci_ && flg_has_extrinsic_bewteen_li_) {
            flg_has_extrinsic_bewteen_ci_ = true;
            quat_c_i_ = quat_c_l_ * quat_l_i_;
            trans_c_i_ = quat_c_l_ * trans_l_i_ + trans_c_l_;
            quat_i_c_ = quat_c_i_.inverse();
            trans_i_c_ = quat_i_c_ * (-trans_c_i_);

            rot_c_i_ = quat_c_i_.toRotationMatrix();
            rot_i_c_ = quat_i_c_.toRotationMatrix();
        }

        if (!flg_has_extrinsic_bewteen_li_ && flg_has_extrinsic_bewteen_ci_) {
            flg_has_extrinsic_bewteen_li_ = true;
            quat_l_i_ = quat_l_c_ * quat_c_i_;
            trans_l_i_ = quat_l_c_ * trans_c_i_ + trans_l_c_;
            quat_i_l_ = quat_l_i_.inverse();
            trans_i_l_ = quat_i_l_ * (-trans_l_i_);

            rot_l_i_ = quat_l_i_.toRotationMatrix();
            rot_i_l_ = quat_i_l_.toRotationMatrix();
        }
    }

    Eigen::Vector3d lidar_2_camera(const Eigen::Vector3d &point)
    {
        return rot_c_l_ * point + trans_c_l_;
    }

    Eigen::Vector3d camera_2_lidar(const Eigen::Vector3d &point)
    {
        return rot_l_c_ * point + trans_l_c_;
    }

    bool flg_has_extrinsic_bewteen_ci_ = false;
    bool flg_has_extrinsic_bewteen_li_ = false;
    bool flg_has_extrinsic_bewteen_cl_ = false;

    Eigen::Quaterniond quat_c_i_;
    Eigen::Matrix3d rot_c_i_;
    Eigen::Vector3d trans_c_i_;

    Eigen::Quaterniond quat_i_c_;
    Eigen::Matrix3d rot_i_c_;
    Eigen::Vector3d trans_i_c_;

    Eigen::Quaterniond quat_l_i_;
    Eigen::Matrix3d rot_l_i_;
    Eigen::Vector3d trans_l_i_;

    Eigen::Quaterniond quat_i_l_;
    Eigen::Matrix3d rot_i_l_;
    Eigen::Vector3d trans_i_l_;

    Eigen::Quaterniond quat_c_l_;
    Eigen::Matrix3d rot_c_l_;
    Eigen::Vector3d trans_c_l_;

    Eigen::Quaterniond quat_l_c_;
    Eigen::Matrix3d rot_l_c_;
    Eigen::Vector3d trans_l_c_;
};