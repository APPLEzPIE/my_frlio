#pragma once
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

/**
 * @brief 相机内参与投影工具。
 *
 * 提供：
 * 1) 从传感器配置文件读取内参与图像尺寸。
 * 2) 相机坐标与像素坐标互相转换。
 * 3) 像素边界判定，避免越界访问图像。
 */
struct Camera_Intrinsic {
public:
    bool load_config_parameter(const std::string &sensor_config_file)
    {
        YAML::Node config_node = YAML::LoadFile(sensor_config_file);

        Eigen::Matrix3d cam_intrinsic_eigen;
        bool has_intrinsic = false;
        try {
            std::vector<double> cam_intrinsic_mat = config_node["cam_intrinsic_mat"].as<std::vector<double>>();
            if (cam_intrinsic_mat.size() == 9) {
                cam_intrinsic_eigen << cam_intrinsic_mat[0], cam_intrinsic_mat[1], cam_intrinsic_mat[2], cam_intrinsic_mat[3], cam_intrinsic_mat[4], cam_intrinsic_mat[5], cam_intrinsic_mat[6],
                    cam_intrinsic_mat[7], cam_intrinsic_mat[8];
                has_intrinsic = true;
            }
        } catch (...) {
        }

        if (!has_intrinsic) {
            try {
                double cam_intrinsic_fx = config_node["cam_intrinsic_fx"].as<double>();
                double cam_intrinsic_fy = config_node["cam_intrinsic_fy"].as<double>();
                double cam_intrinsic_cx = config_node["cam_intrinsic_cx"].as<double>();
                double cam_intrinsic_cy = config_node["cam_intrinsic_cy"].as<double>();
                cam_intrinsic_eigen << cam_intrinsic_fx, 0.0, cam_intrinsic_cx, 0.0, cam_intrinsic_fy, cam_intrinsic_cy, 0.0, 0.0, 1.0;
                has_intrinsic = true;
            } catch (...) {
            }
        }

        if (!has_intrinsic) {
            LOG(WARNING) << "[Camera_Intrinsic]: missing camera intrinsic in " << sensor_config_file;
            return false;
        }

        Eigen::Matrix<double, 5, 1> cam_dist_coeffs_eigen;
        try {
            std::vector<double> cam_dist_coeffs_vec = config_node["cam_dist_coeffs_vec"].as<std::vector<double>>();
            if (cam_dist_coeffs_vec.size() >= 5) {
                cam_dist_coeffs_eigen << cam_dist_coeffs_vec[0], cam_dist_coeffs_vec[1], cam_dist_coeffs_vec[2], cam_dist_coeffs_vec[3], cam_dist_coeffs_vec[4];
            } else {
                cam_dist_coeffs_eigen.setZero();
            }
        } catch (...) {
            cam_dist_coeffs_eigen.setZero();
        }

        int width = -1;
        int height = -1;
        try {
            width = config_node["cam_width"].as<int>();
            height = config_node["cam_height"].as<int>();
        } catch (...) {
            LOG(WARNING) << "[Camera_Intrinsic]: missing camera size in " << sensor_config_file;
        }

        set_parameter(width, height, cam_intrinsic_eigen, cam_dist_coeffs_eigen);
        return true;
    }

    void set_parameter(int width, int height, const Eigen::Matrix3d &cam_intrinsic_eigen, const Eigen::Matrix<double, 5, 1> &cam_dist_coeffs_eigen)
    {
        width_ = width;
        height_ = height;
        mat_ = cam_intrinsic_eigen;
        dist_coeffs_ = cam_dist_coeffs_eigen;

        fx_ = mat_(0, 0);
        fy_ = mat_(1, 1);
        cx_ = mat_(0, 2);
        cy_ = mat_(1, 2);
    }

    void set_parameter(const Eigen::Matrix3d &cam_intrinsic_eigen, const Eigen::Matrix<double, 5, 1> &cam_dist_coeffs_eigen)
    {
        mat_ = cam_intrinsic_eigen;
        dist_coeffs_ = cam_dist_coeffs_eigen;

        fx_ = mat_(0, 0);
        fy_ = mat_(1, 1);
        cx_ = mat_(0, 2);
        cy_ = mat_(1, 2);
    }

    bool in_border(const double &u, const double &v, const int &boundary_size = 0)
    {
        return boundary_size <= u && u < width_ - boundary_size && boundary_size <= v && v < height_ - boundary_size;
    }

    bool in_border(const int &u, const int &v, const int &boundary_size = 0)
    {
        return boundary_size <= u && u < width_ - boundary_size && boundary_size <= v && v < height_ - boundary_size;
    }

    void camera_2_pixel(const Eigen::Vector3d &point_camera, double &pixel_u, double &pixel_v)
    {
        double x = point_camera[0] / point_camera[2];
        double y = point_camera[1] / point_camera[2];
        double r2 = x * x + y * y;
        double r4 = r2 * r2;
        double r6 = r4 * r2;
        double a1 = 2.0 * x * y;
        double a2 = r2 + 2.0 * x * x;
        double a3 = r2 + 2.0 * y * y;
        double cdist = 1.0 + dist_coeffs_[0] * r2 + dist_coeffs_[1] * r4 + dist_coeffs_[4] * r6;
        double xd = x * cdist + dist_coeffs_[2] * a1 + dist_coeffs_[3] * a2;
        double yd = y * cdist + dist_coeffs_[2] * a3 + dist_coeffs_[3] * a1;
        pixel_u = xd * fx_ + cx_;
        pixel_v = yd * fy_ + cy_;
    }

    void pixel_2_camera(const double &pixel_u, const double &pixel_v, Eigen::Vector3d &point_camera)
    {
        point_camera[0] = (pixel_u - cx_) / fx_;
        point_camera[1] = (pixel_v - cy_) / fy_;
        point_camera[2] = 1.0;
    }

    int get_width()
    {
        return width_;
    }

    int get_height()
    {
        return height_;
    }

    void dpi(const Eigen::Vector3d &camera_point, Eigen::Matrix<double, 2, 3> &jacbian)
    {
        const double z_inv = 1.0 / camera_point[2];
        const double z_inv_2 = z_inv * z_inv;
        jacbian(0, 0) = fx_ * z_inv;
        jacbian(0, 1) = 0.0;
        jacbian(0, 2) = -fx_ * camera_point[0] * z_inv_2;
        jacbian(1, 0) = 0.0;
        jacbian(1, 1) = fy_ * z_inv;
        jacbian(1, 2) = -fy_ * camera_point[1] * z_inv_2;
    }

private:
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    Eigen::Matrix3d mat_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 5, 1> dist_coeffs_ = Eigen::Matrix<double, 5, 1>::Zero();
    int width_ = -1;
    int height_ = -1;
};
