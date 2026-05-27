#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "../tools/math_tools.hpp"

/**
 * @brief 系统状态容器（姿态、位置、速度、偏置、重力等）。
 *
 * 为滤波/优化提供统一状态表达以及流形上的加减法操作。
 */
// 滤波+优化
// 滤波：处理短时的状态估计，保证里程计的快速性（预测+更新+平滑）
// 优化：滑窗的形式，或者所有帧，进行局部优化以及全局优化

// 滑窗
// 维护一个滑窗
// 滑窗的作用1：保存上一个未退化帧到下一个未退化帧，以及中间的帧，进行一次rts后将第二个未退化帧之前的帧都丢弃（以此处理退化）
// 滑窗的作用2：通过一个更大的窗口来联合优化IMU的一些状态量
// 滑窗的作用3：以图优化的方式在滑窗中优化系统状态以及路标
//
// 状态包括
// qpv,ba,bg,grav,ext
// 其中有静态的
// 系统状态， 提供
// 流形上的加法，减法操作
// 赋值操作
// eskf的预测更新基于这个去做

#define DIM_OF_STATES (18)

class State_Group {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    template <typename T> Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R)
    {
        T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
        Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
        return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
    }

    // template<typename T>
    // Eigen::Matrix<double, 3, 3> Exp(const double &v1, const double &v2, const double &v3)
    // {
    //     double &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    //     Eigen::Matrix<double, 3, 3> Eye3 = Eigen::Matrix<double, 3, 3>::Identity();
    //     if (norm > 0.00001)
    //     {
    //         Eigen::Matrix<double,3,1> r_ang(v1 / norm, v2 / norm, v3 / norm);
    //         Eigen::Matrix<double, 3, 3> K;
    //         K = MathTools::SkewSymmetric(r_ang);

    //         /// Roderigous Tranformation
    //         return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    //     }
    //     else
    //     {
    //         return Eye3;
    //     }
    // }

    Eigen::Matrix<double, DIM_OF_STATES, 1> operator-(State_Group &state_1)
    {
        Eigen::Matrix<double, DIM_OF_STATES, 1> dx;

        dx.block<3, 1>(0, 0) = pos_ - state_1.pos_;
        dx.block<3, 1>(3, 0) = vel_ - state_1.vel_;
        dx.block<3, 1>(6, 0) = Log((state_1.quat_.inverse() * quat_).toRotationMatrix());
        // dx.block<3,1>(6,0) = 2*(state_1.quat_.inverse()*quat_).vec();
        dx.block<3, 1>(9, 0) = ba_ - state_1.ba_;
        dx.block<3, 1>(12, 0) = bg_ - state_1.bg_;
        dx.block<3, 1>(15, 0) = grav_ - state_1.grav_;

        return dx;
    }

    State_Group operator+(State_Group &)
    {
        Eigen::Matrix<double, DIM_OF_STATES, 1> dx;
        State_Group state_tmp = *this;
        return state_tmp += dx;
    }

    State_Group operator+(const Eigen::Matrix<double, DIM_OF_STATES, 1> &dx)
    {
        State_Group state_tmp = *this;
        return state_tmp += dx;
    }

    State_Group &operator+=(const Eigen::Matrix<double, DIM_OF_STATES, 1> &dx)
    {
        pos_ += dx.block<3, 1>(0, 0);
        vel_ += dx.block<3, 1>(3, 0);
        // quat_ = Eigen::Quaterniond(quat_.toRotationMatrix * Exp(dx.block<3,1>(6,0)(0,0), dx.block<3,1>(6,0)(1,0), dx.block<3,1>(6,0)(2,0)));
        quat_ = (quat_ * MathTools::DeltaQ(dx.block<3, 1>(6, 0)));
        quat_.normalize();
        ba_ += dx.block<3, 1>(9, 0);
        bg_ += dx.block<3, 1>(12, 0);
        grav_ += dx.block<3, 1>(15, 0);
        return *this;
    }

    void print()
    {
        std::cout << "[state]" << std::endl
                  << "[pos]:" << pos_.transpose() << std::endl
                  << "[vel]:" << vel_.transpose() << std::endl
                  << "[quat]:" << quat_.toRotationMatrix().eulerAngles(0, 1, 2).transpose() << std::endl
                  << "[ba]:" << ba_.transpose() << std::endl
                  << "[bg]:" << bg_.transpose() << std::endl
                  << "[grav]:" << grav_.transpose() << std::endl;
    }
    double timestamp_;

    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Quaterniond quat_;
    Eigen::Vector3d ba_;
    Eigen::Vector3d bg_;
    Eigen::Vector3d grav_;

    Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> cov_;
};