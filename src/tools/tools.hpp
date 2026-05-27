#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>

/**
 * @brief 通用工具函数集合。
 *
 * 目前包含轨迹导出、几何估计等与算法逻辑解耦的辅助函数。
 */
class Tools {
public:
    static void save_tum_trajectory(const std::string &traj_file, const std::vector<double> &traj_time, const std::vector<Eigen::Quaterniond> &traj_quat, const std::vector<Eigen::Vector3d> &traj_pos)
    {
        std::ofstream ofs;
        ofs.open(traj_file, std::ios::out);
        if (!ofs.is_open()) {
            std::cout << "Failed to open traj_file: " << traj_file;
            return;
        }

        ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
        for (int i = 0; i < traj_time.size(); i++) {
            ofs << std::fixed << std::setprecision(6) << traj_time[i] << " " << std::setprecision(15) << traj_pos[i].x() << " " << traj_pos[i].y() << " " << traj_pos[i].z() << " " << traj_quat[i].x()
                << " " << traj_quat[i].y() << " " << traj_quat[i].z() << " " << traj_quat[i].w() << std::endl;
        }

        ofs.close();
    }

    template <typename PointType>
    static bool esti_plane(Eigen::Matrix<float, 4, 1> &pca_result, const std::vector<PointType, Eigen::aligned_allocator<PointType>> &point, const float &threshold = 0.1f)
    {
        if (point.size() < 3) {
            return false;
        }
        Eigen::Matrix<float, 3, 1> normvec;

        if (point.size() == 5) {
            Eigen::Matrix<float, 5, 3> A;
            Eigen::Matrix<float, 5, 1> b;

            A.setZero();
            b.setOnes();
            b *= -1.0f;

            for (int j = 0; j < 5; j++) {
                A(j, 0) = point[j].x;
                A(j, 1) = point[j].y;
                A(j, 2) = point[j].z;
            }

            normvec = A.colPivHouseholderQr().solve(b);
        } else {
            Eigen::MatrixXf A(point.size(), 3);
            Eigen::VectorXf b(point.size(), 1);

            A.setZero();
            b.setOnes();
            b *= -1.0f;

            for (int j = 0; j < point.size(); j++) {
                A(j, 0) = point[j].x;
                A(j, 1) = point[j].y;
                A(j, 2) = point[j].z;
            }

            Eigen::MatrixXf n = A.colPivHouseholderQr().solve(b);
            normvec(0, 0) = n(0, 0);
            normvec(1, 0) = n(1, 0);
            normvec(2, 0) = n(2, 0);
        }

        float n = normvec.norm();
        pca_result(0) = normvec(0) / n;
        pca_result(1) = normvec(1) / n;
        pca_result(2) = normvec(2) / n;
        pca_result(3) = 1.0 / n;

        for (const auto &p : point) {
            Eigen::Matrix<float, 4, 1> temp;
            temp.block<3, 1>(0, 0) << p.x, p.y, p.z;
            temp[3] = 1.0;
            if (fabs(pca_result.dot(temp)) > threshold) {
                return false;
            }
        }

        return true;
    }

    template <typename PointType>
    static bool esti_plane(Eigen::Matrix<double, 3, 1> &pca_center, Eigen::Matrix<double, 3, 1> &pca_normal, const std::vector<PointType, Eigen::aligned_allocator<PointType>> &point,
                           const double &threshold = 0.1f)
    {
        if (point.size() < 3) {
            return false;
        }
        Eigen::Matrix<double, 4, 1> pca_result;
        Eigen::Matrix<double, 3, 1> normvec;

        pca_center.setZero();
        if (point.size() == 5) {
            Eigen::Matrix<double, 5, 3> A;
            Eigen::Matrix<double, 5, 1> b;

            A.setZero();
            b.setOnes();
            b *= -1.0f;

            for (int j = 0; j < 5; j++) {
                A(j, 0) = point[j].x;
                A(j, 1) = point[j].y;
                A(j, 2) = point[j].z;

                pca_center += Eigen::Matrix<double, 3, 1>(point[j].x, point[j].y, point[j].z);
            }

            normvec = A.colPivHouseholderQr().solve(b);
        } else {
            Eigen::MatrixXd A(point.size(), 3);
            Eigen::VectorXd b(point.size(), 1);

            A.setZero();
            b.setOnes();
            b *= -1.0f;

            for (int j = 0; j < point.size(); j++) {
                A(j, 0) = point[j].x;
                A(j, 1) = point[j].y;
                A(j, 2) = point[j].z;

                pca_center += Eigen::Matrix<double, 3, 1>(point[j].x, point[j].y, point[j].z);
            }

            Eigen::MatrixXd n = A.colPivHouseholderQr().solve(b);
            normvec(0, 0) = n(0, 0);
            normvec(1, 0) = n(1, 0);
            normvec(2, 0) = n(2, 0);
        }

        double n = normvec.norm();
        pca_result(0) = normvec(0) / n;
        pca_result(1) = normvec(1) / n;
        pca_result(2) = normvec(2) / n;
        pca_result(3) = 1.0 / n;

        for (const auto &p : point) {
            Eigen::Matrix<double, 4, 1> temp;
            temp.block<3, 1>(0, 0) << p.x, p.y, p.z;
            temp[3] = 1.0;
            if (fabs(pca_result.dot(temp)) > threshold) {
                return false;
            }
        }

        pca_normal = pca_result.block<3, 1>(0, 0);
        pca_center /= point.size();
        return true;
    }

    struct hash_vec {
        inline size_t operator()(const Eigen::Matrix<int, 3, 1> &v) const
        {
            return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^ ((v[2]) * 83492791)) % 10000000;
        }
    };

    template <typename PointType> static typename pcl::PointCloud<PointType>::Ptr downsample_voxel_filter(typename pcl::PointCloud<PointType>::Ptr cloud_in, float voxel_size)
    {
        typename pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>());
        if (voxel_size < 0.01) {
            *cloud_out = *cloud_in;
            return cloud_out;
        }
        cloud_out->reserve(cloud_in->size());
        std::unordered_set<Eigen::Vector3i, hash_vec> voxel_map;
        Eigen::Vector3i key;
        Eigen::Vector3d pt;
        int ds_size = 0;
        for (auto &point : cloud_in->points) {
            pt << point.x, point.y, point.z;
            key = (pt / voxel_size).array().round().template cast<int>();
            auto iter = voxel_map.find(key);
            if (iter == voxel_map.end()) {
                voxel_map.insert(key);
                cloud_out->push_back(point);
            }
        }
        return cloud_out;
    }
};
