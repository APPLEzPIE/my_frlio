#pragma once
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief 数学工具集合。
 *
 * 提供SO(3)相关常用操作（反对称矩阵、指数映射等），
 * 用于IMU预积分、姿态更新和误差状态计算。
 */
class MathTools {
public:
    template <typename Derived> static Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1), q(2), typename Derived::Scalar(0), -q(0), -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }

    template <typename T> static Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang)
    {
        T ang_norm = ang.norm();
        Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
        if (ang_norm > 0.0000001) {
            Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
            Eigen::Matrix<T, 3, 3> K;
            K = MathTools::SkewSymmetric(r_axis);
            /// Roderigous Tranformation
            return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
        } else {
            return Eye3;
        }
    }

    template <typename T, typename Ts> static Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
    {
        T ang_vel_norm = ang_vel.norm();
        Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

        if (ang_vel_norm > 0.0000001) {
            Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
            Eigen::Matrix<T, 3, 3> K;

            K = MathTools::SkewSymmetric(r_axis);

            T r_ang = ang_vel_norm * dt;

            /// Roderigous Tranformation
            return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
        } else {
            return Eye3;
        }
    }
    template <typename T, typename Ts> static Eigen::Quaternion<T> Exp_Quat(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
    {
        return Eigen::Quaternion<T>(1, ang_vel[0] * dt / 2.0, ang_vel[1] * dt / 2.0, ang_vel[2] * dt / 2.0);
    }

    template <typename T> static Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
    {
        T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
        Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
        if (norm > 0.00001) {
            T r_ang[3] = { v1 / norm, v2 / norm, v3 / norm };
            Eigen::Matrix<T, 3, 3> K;
            K = MathTools::SkewSymmetric(r_ang);

            /// Roderigous Tranformation
            return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
        } else {
            return Eye3;
        }
    }

    /* Logrithm of a Rotation Matrix */
    template <typename T> static Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R)
    {
        T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
        Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
        return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
    }

    template <typename Derived> static Eigen::Quaternion<typename Derived::Scalar> DeltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

    template <typename Derived> static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &qq)
    {
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + MathTools::SkewSymmetric(qq.vec());
        return ans;
    }

    template <typename Derived> static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &pp)
    {
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - MathTools::SkewSymmetric(pp.vec());
        return ans;
    }
};