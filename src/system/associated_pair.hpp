#pragma once

#include "../tools/math_tools.hpp"

template <typename PclPointType> class Associated_Pair {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Associated_Pair()
        : flg_valid_(false)
    {
    }

    bool flg_valid_;
    bool flg_valid1_;
    Eigen::Vector3d pt_;
    Eigen::Matrix3d pt_cov_;

    Eigen::Vector3d feature_center_;
    Eigen::Matrix3d feature_center_cov_;
    Eigen::Vector3d feature_normal_;
    Eigen::Matrix3d feature_normal_cov_;
    std::vector<PclPointType, Eigen::aligned_allocator<PclPointType>> nearest_points_;

    double cov_;
    double residual_;
    Eigen::Matrix<double, 1, 6> jacobian_;
    Eigen::Matrix<double, 1, 6> jacobian_proj_;
    Eigen::Matrix<float, 4, 1> plane_coef_;
};