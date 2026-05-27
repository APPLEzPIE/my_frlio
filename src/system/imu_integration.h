#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "../basic_element/state_group.hpp"
#include "../sensor/sensor_data.hpp"
#include "../tools/math_tools.hpp"

/**
 * @brief IMU积分与误差状态更新模块。
 *
 * 职责：
 * 1. 根据IMU序列进行状态传播（姿态/速度/位置与协方差）。
 * 2. 接收外部观测执行迭代ESKF更新。
 * 3. 管理IMU噪声参数与初始化状态。
 */
class Imu_Integration {
public:
    Imu_Integration() = default;
    Imu_Integration(const std::deque<std::shared_ptr<Imu_Data>> &imu_datas);
    Imu_Integration(const std::deque<std::shared_ptr<Imu_Data>> &imu_datas, bool flg_propagate);

    static void set_init_state(const State_Group &init_state);
    static void load_imu_noise(const std::string &sensor_config_file_);
    static void set_imu_noise_std(const double &acc_std, const double &gyr_std, const double &acc_bias_std, const double &gyr_bias_std);
    static void set_imu_noise_std_init(const double &acc_std, const double &gyr_std, const double &acc_bias_std, const double &gyr_bias_std);

    static State_Group &get_curr_system_state()
    {
        return system_state_;
    }

    void add_meas(const std::deque<std::shared_ptr<Imu_Data>> &imu_data_vec);
    void propagate();
    void update(const Eigen::Vector3d &obs_pos, const Eigen::Quaterniond &obs_quat, const Eigen::Matrix<double, 6, 6> &obs_cov);
    void ieskf_update(const Eigen::MatrixXd &Hsub, const Eigen::MatrixXd &res_vec, const Eigen::MatrixXd &cov, int res_num, bool flg_final_iter);
    bool ieskf_update_state(const Eigen::MatrixXd &Hsub, const Eigen::MatrixXd &res_vec, const Eigen::MatrixXd &cov_i, int res_num, const double &position_converage_thres = 0.00015,
                            const double &angle_converage_thres = 0.01);
    void ieskf_update_cov_finally();
    void smooth();

    std::vector<State_Group> &get_high_freq_states()
    {
        return temp_states_;
    }
    State_Group &get_curr_state()
    {
        return temp_states_.back();
    }

    static double get_imu_noise_acc_std_init()
    {
        return imu_noise_acc_std_init_;
    }
    static double get_imu_noise_gyr_std_init()
    {
        return imu_noise_gyr_std_init_;
    }
    static double get_imu_noise_acc_std()
    {
        return imu_noise_acc_std_;
    }
    static double get_imu_noise_gyr_std()
    {
        return imu_noise_gyr_std_;
    }

private:
    // 输入
    // time_buf_中保存的是相对每帧第一个点的时间,
    //第一帧IMU时间戳与当前LiDAR帧中第一个点时间戳相同（确切的说应该是与上一LiDAR帧中最后一个点时间戳相同）,
    //最后一帧IMU时间戳与当前LiDAR帧中最后一个点时间戳相同。 因此至少有3个IMU数据（第一个和最后一个是插值得到的）。
    // acc_buf_ 、 gyr_buf_ 中是与 time_buf_ 对应的IMU数据。
    std::size_t data_size_;
    std::vector<double> time_buf_;
    std::vector<Eigen::Vector3d> acc_buf_;
    std::vector<Eigen::Vector3d> gyr_buf_;

    // IMU频率的位姿数据
    std::vector<State_Group> temp_states_;

    // 系统整体的状态变量
    static State_Group system_state_;
    static State_Group system_state_temp_for_smooth_;
    // IMU输入噪声
    static Eigen::Matrix<double, 12, 12> imu_noise_cov_; // acc（3）、gyr（3）、acc（3）、gyr（3）、ba（3）、bg（3）
    static double imu_noise_acc_std_;
    static double imu_noise_gyr_std_;
    static double imu_noise_ba_std_;
    static double imu_noise_bg_std_;
    static double imu_noise_acc_std_init_;
    static double imu_noise_gyr_std_init_;
    static double imu_noise_ba_std_init_;
    static double imu_noise_bg_std_init_;

    // ieskf更新时计算中间值保存
    Eigen::Matrix<double, 18, 18> KH_tmp_;
};

State_Group Imu_Integration::system_state_;
State_Group Imu_Integration::system_state_temp_for_smooth_;
Eigen::Matrix<double, 12, 12> Imu_Integration::imu_noise_cov_; // acc（3）、gyr（3）、ba（3）、bg（3）
double Imu_Integration::imu_noise_acc_std_;
double Imu_Integration::imu_noise_gyr_std_;
double Imu_Integration::imu_noise_ba_std_;
double Imu_Integration::imu_noise_bg_std_;
double Imu_Integration::imu_noise_acc_std_init_;
double Imu_Integration::imu_noise_gyr_std_init_;
double Imu_Integration::imu_noise_ba_std_init_;
double Imu_Integration::imu_noise_bg_std_init_;

Imu_Integration::Imu_Integration(const std::deque<std::shared_ptr<Imu_Data>> &imu_datas, bool flg_propagate)
{
    add_meas(imu_datas);
    if (flg_propagate) {
        propagate();
    }
}

Imu_Integration::Imu_Integration(const std::deque<std::shared_ptr<Imu_Data>> &imu_datas)
{
    add_meas(imu_datas);
    propagate();
}

void Imu_Integration::set_init_state(const State_Group &init_state)
{
    system_state_ = init_state;
}

void Imu_Integration::add_meas(const std::deque<std::shared_ptr<Imu_Data>> &imu_data_vec)
{
    for (auto imu_iter = imu_data_vec.begin(); imu_iter != imu_data_vec.end(); imu_iter++) {
        std::shared_ptr<Imu_Data> imu_data = *imu_iter;
        time_buf_.push_back(imu_data->timestamp_);
        acc_buf_.push_back(imu_data->acc_);
        gyr_buf_.push_back(imu_data->gyr_);
    }
    data_size_ = time_buf_.size();
}

void Imu_Integration::load_imu_noise(const std::string &sensor_config_file_)
{
    YAML::Node config_node = YAML::LoadFile(sensor_config_file_);
    double acc_std, gyr_std, ba_std, bg_std;
    try {
        acc_std = config_node["imu_noise"]["acc_std"].as<double>();
        gyr_std = config_node["imu_noise"]["gyr_std"].as<double>();
        ba_std = config_node["imu_noise"]["ba_std"].as<double>();
        bg_std = config_node["imu_noise"]["bg_std"].as<double>();
    } catch (...) {
        LOG(FATAL) << "[Imu_Integration]: file " << sensor_config_file_ << " has a bad conversion";
    }

    LOG(INFO) << std::endl
              << "| --------------- [Imu_Integration parameter] --------------- |" << std::endl
              << "[acc_std] " << acc_std << std::endl
              << "[gyr_std] " << gyr_std << std::endl
              << "[ba_std] " << ba_std << std::endl
              << "[bg_std] " << bg_std << std::endl
              << std::endl;

    Imu_Integration::set_imu_noise_std_init(acc_std, gyr_std, ba_std, bg_std);
}

void Imu_Integration::set_imu_noise_std(const double &acc_std, const double &gyr_std, const double &ba_std, const double &bg_std)
{
    Imu_Integration::imu_noise_acc_std_ = acc_std;
    Imu_Integration::imu_noise_gyr_std_ = gyr_std;
    Imu_Integration::imu_noise_ba_std_ = ba_std;
    Imu_Integration::imu_noise_bg_std_ = bg_std;

    imu_noise_cov_ = Eigen::Matrix<double, 12, 12>::Zero();
    imu_noise_cov_.block<3, 3>(0, 0) = Eigen::Vector3d(acc_std * acc_std, acc_std * acc_std, acc_std * acc_std).asDiagonal();
    imu_noise_cov_.block<3, 3>(3, 3) = Eigen::Vector3d(gyr_std * gyr_std, gyr_std * gyr_std, gyr_std * gyr_std).asDiagonal();
    imu_noise_cov_.block<3, 3>(6, 6) = Eigen::Vector3d(ba_std * ba_std, ba_std * ba_std, ba_std * ba_std).asDiagonal();
    imu_noise_cov_.block<3, 3>(9, 9) = Eigen::Vector3d(bg_std * bg_std, bg_std * bg_std, bg_std * bg_std).asDiagonal();
}

void Imu_Integration::set_imu_noise_std_init(const double &acc_std, const double &gyr_std, const double &ba_std, const double &bg_std)
{
    Imu_Integration::imu_noise_acc_std_ = acc_std;
    Imu_Integration::imu_noise_gyr_std_ = gyr_std;
    Imu_Integration::imu_noise_ba_std_ = ba_std;
    Imu_Integration::imu_noise_bg_std_ = bg_std;
    Imu_Integration::imu_noise_acc_std_init_ = acc_std;
    Imu_Integration::imu_noise_gyr_std_init_ = gyr_std;
    Imu_Integration::imu_noise_ba_std_init_ = ba_std;
    Imu_Integration::imu_noise_bg_std_init_ = bg_std;

    imu_noise_cov_ = Eigen::Matrix<double, 12, 12>::Zero();
    imu_noise_cov_.block<3, 3>(0, 0) = Eigen::Vector3d(acc_std * acc_std, acc_std * acc_std, acc_std * acc_std).asDiagonal();
    imu_noise_cov_.block<3, 3>(3, 3) = Eigen::Vector3d(gyr_std * gyr_std, gyr_std * gyr_std, gyr_std * gyr_std).asDiagonal();
    imu_noise_cov_.block<3, 3>(6, 6) = Eigen::Vector3d(ba_std * ba_std, ba_std * ba_std, ba_std * ba_std).asDiagonal();
    imu_noise_cov_.block<3, 3>(9, 9) = Eigen::Vector3d(bg_std * bg_std, bg_std * bg_std, bg_std * bg_std).asDiagonal();
}

void Imu_Integration::propagate()
{
    // 为每个 IMU 时刻分配一个状态槽位（与 time_buf_ 一一对应）
    temp_states_.resize(data_size_);
    // 第一个状态直接取当前系统状态，作为传播起点
    temp_states_[0] = system_state_;

    // 相邻 IMU 样本时间间隔
    double dt;
    // 无偏加速度/角速度相关中间量（梯形积分）
    Eigen::Vector3d un_acc, un_acc_0, un_acc_1, un_gyr;
    // 离散状态转移雅可比 F（状态维度18：p,v,q,ba,bg,g）
    Eigen::Matrix<double, 18, 18> F; // p（3）、v（3）、q（3）、ba（3）、bg（3）
    // 过程噪声映射矩阵 V（噪声维度12：acc,gyr,ba,bg）
    Eigen::Matrix<double, 18, 12> V; // acc（3）、gyr（3）、ba（3）、bg（3）

    // 从第2个 IMU 样本开始逐步向前传播状态
    for (int idx = 1; idx < data_size_; idx++) {
        // state_1 为要预测的帧， state_0 为前一帧
        State_Group &state_0 = temp_states_[idx - 1];
        State_Group &state_1 = temp_states_[idx];

        // -------------------- 1) 状态预测 --------------------
        // 当前状态对应时间戳
        state_1.timestamp_ = time_buf_[idx];
        // 两个 IMU 样本之间的时间间隔
        dt = time_buf_[idx] - time_buf_[idx - 1];

        // 去偏后的角速度（使用前后时刻均值，降低离散误差）
        un_gyr = 0.5 * (gyr_buf_[idx - 1] + gyr_buf_[idx]) - state_0.bg_;
        // 姿态积分：q_k = q_{k-1} * Exp(w*dt)
        state_1.quat_ = state_0.quat_ * /* MathTools::Exp(un_gyr, dt) */ MathTools::Exp_Quat(un_gyr, dt);

        // 前一时刻去偏加速度，旋转到世界系
        un_acc_0 = state_0.quat_ * (acc_buf_[idx - 1] - state_0.ba_);
        // 当前时刻去偏加速度，旋转到世界系
        un_acc_1 = state_1.quat_ * (acc_buf_[idx] - state_0.ba_);
        // 梯形积分的平均加速度（重力在位置/速度更新时再显式加入）
        un_acc = 0.5 * (un_acc_0 + un_acc_1) /*   + system_gravity_ */;

        // 位置更新：p_k = p_{k-1} + v_{k-1}*dt + 0.5*(a+g)*dt^2
        state_1.pos_ = state_0.pos_ + state_0.vel_ * dt + 0.5 * (un_acc + state_0.grav_) * dt * dt;
        // 速度更新：v_k = v_{k-1} + (a+g)*dt
        state_1.vel_ = state_0.vel_ + (un_acc + state_0.grav_) * dt;
        // 偏置和重力在传播阶段按常值模型保持不变
        state_1.ba_ = state_0.ba_;
        state_1.bg_ = state_0.bg_;
        state_1.grav_ = state_0.grav_;

        // -------------------- 2) 协方差预测 --------------------
        // 线性化状态转移矩阵 F
        F = Eigen::Matrix<double, 18, 18>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt; // dp_dv
        F.block<3, 3>(3, 6) = -state_0.quat_.toRotationMatrix() * MathTools::SkewSymmetric(un_acc) * dt; // dv_dq
        F.block<3, 3>(3, 9) = -state_0.quat_.toRotationMatrix() * dt; // dv_dba
        F.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt; // dv_dg
        F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - MathTools::SkewSymmetric(un_gyr) * dt; // dq_dq
        F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt; // dq_dbg

        // 过程噪声注入矩阵 V（将连续噪声离散化到状态）
        V = Eigen::Matrix<double, 18, 12>::Zero();
        V.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * dt;
        V.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;
        V.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity() * sqrt(dt);
        V.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * sqrt(dt);

        // 协方差离散传播：P_k = F*P_{k-1}*F^T + V*Q*V^T
        state_1.cov_ = F * state_0.cov_ * F.transpose() + V * imu_noise_cov_ * V.transpose();
    }

    // 用本次传播末状态刷新“系统当前状态”
    system_state_ = temp_states_.back();
    // 同步平滑起点状态
    system_state_temp_for_smooth_ = system_state_;
}

void Imu_Integration::update(const Eigen::Vector3d &obs_pos, const Eigen::Quaterniond &obs_quat, const Eigen::Matrix<double, 6, 6> &obs_cov)
{
    Eigen::Matrix3d state_rot_inv = temp_states_.back().quat_.toRotationMatrix().transpose();
    Eigen::Quaterniond q_e = temp_states_.back().quat_.conjugate() * obs_quat;

    Eigen::Matrix<double, 6, 1> z;
    z.setZero();

    Eigen::Matrix<double, 6, 1> h;
    h.block<3, 1>(0, 0) = state_rot_inv * (obs_pos - temp_states_.back().pos_); // 平移e
    h.block<3, 1>(3, 0) = 2.0 * q_e.vec(); // 旋转e

    Eigen::Matrix<double, 6, 6> h_v;
    h_v.setZero();
    h_v.block<3, 3>(0, 0) = state_rot_inv; // 平移e-平移n
    h_v.block<3, 3>(3, 3) = MathTools::Qleft(q_e).block<3, 3>(1, 1); // 旋转e-旋转n

    Eigen::Matrix<double, 6, 18> h_x;
    h_x.setZero();
    h_x.block<3, 3>(0, 0) = -state_rot_inv; // 平移e---平移x
    h_x.block<3, 3>(0, 6) = MathTools::SkewSymmetric(h.block<3, 1>(0, 0)); // 平移e---旋转x
    h_x.block<3, 3>(3, 6) = -MathTools::Qright(q_e).block<3, 3>(1, 1); // 旋转e---旋转x

    Eigen::Matrix<double, 18, 6> K;
    K = temp_states_.back().cov_ * h_x.transpose() *
        (h_x * temp_states_.back().cov_ * h_x.transpose() + h_v * obs_cov /* (Eigen::Matrix<double, 6, 6>::Identity()*0.00001*0.00001) */ * h_v.transpose()).inverse();

    Eigen::Matrix<double, 18, 1> dx = K * (z - h);
    temp_states_.back().cov_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * h_x) * temp_states_.back().cov_;
    temp_states_.back() += dx;

    system_state_ = temp_states_.back();
    system_state_temp_for_smooth_ = system_state_;
}

bool Imu_Integration::ieskf_update_state(const Eigen::MatrixXd &Hsub, const Eigen::MatrixXd &res_vec, const Eigen::MatrixXd &cov_i, int res_num, const double &position_converage_thres,
                                         const double &angle_converage_thres)
{
    Eigen::Matrix<double, 18, 18> G, H_T_H, I_STATE, K_1;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    Eigen::MatrixXd Hsub_T = Hsub.transpose(); // H转置 : 6xn = (nx6)^T

    Eigen::MatrixXd HsubTCI = Hsub_T;
    for (int i = 0; i < res_num; i++) {
        HsubTCI.col(i) *= cov_i(i, 0);
    }

    Eigen::Matrix<double, 6, 6> H_T_H_Sub = HsubTCI * Hsub;
    H_T_H.block<3, 3>(0, 0) = H_T_H_Sub.block<3, 3>(0, 0);
    H_T_H.block<3, 3>(6, 6) = H_T_H_Sub.block<3, 3>(3, 3);
    H_T_H.block<3, 3>(0, 6) = H_T_H_Sub.block<3, 3>(0, 3);
    H_T_H.block<3, 3>(6, 0) = H_T_H_Sub.block<3, 3>(3, 0);

    Eigen::Matrix<double, 6, 1> HTz = HsubTCI * res_vec;

    Eigen::Matrix<double, 18, 1> HTCI; // n*1
    HTCI.setZero();
    HTCI.block<3, 1>(0, 0) = HTz.block<3, 1>(0, 0);
    HTCI.block<3, 1>(6, 0) = HTz.block<3, 1>(3, 0);

    K_1 = (H_T_H + system_state_.cov_.inverse()).inverse();
    KH_tmp_ = K_1 * H_T_H;

    Eigen::Matrix<double, 18, 1> vec = temp_states_.back() - system_state_;

    Eigen::Matrix<double, 18, 1> dx = -(K_1 * HTCI) - vec + KH_tmp_ * vec; // 最后发现应该是忘记了负号，修改后正确就没有问题了。
    // Eigen::Matrix<double, 18, 1> dx = - (K_1 * HTCI); // 效果与上式基本相同

    temp_states_.back() += dx;
    temp_states_.back().cov_ = (Eigen::Matrix<double, 18, 18>::Identity() - KH_tmp_) * system_state_.cov_;

    system_state_temp_for_smooth_ = temp_states_.back();

    if ((dx.block<3, 1>(0, 0).norm() < position_converage_thres) && (dx.block<3, 1>(6, 0).norm() * 57.29578 < angle_converage_thres)) {
        return true;
    } else {
        return false;
    }
}

void Imu_Integration::ieskf_update_cov_finally()
{
    temp_states_.back().cov_ = (Eigen::Matrix<double, 18, 18>::Identity() - KH_tmp_) * system_state_.cov_;
    system_state_ = temp_states_.back();
    system_state_temp_for_smooth_ = system_state_;
}

void Imu_Integration::ieskf_update(const Eigen::MatrixXd &Hsub, const Eigen::MatrixXd &res_vec, const Eigen::MatrixXd &cov_i, int res_num, bool flg_final_iter)
{
    Eigen::Matrix<double, 18, 18> G, H_T_H, I_STATE, K_1;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();
    // cout << ANSI_COLOR_RED_BOLD << "Run EKF uph" << ANSI_COLOR_RESET << endl;
    // 1>:求公式中需要用到的 H 和 H^T*T

    auto Hsub_T = Hsub.transpose(); // H转置 : 6xn = (nx6)^T

    Eigen::MatrixXd HsubTCI = Hsub_T;
    for (int i = 0; i < res_num; i++) {
        HsubTCI.col(i) *= cov_i(i, i);
    }

    Eigen::Matrix<double, 6, 6> H_T_H_Sub = HsubTCI * Hsub;

    // std::cout << "ieskf_update 1 1" << std::endl;
    H_T_H.block<3, 3>(0, 0) = H_T_H_Sub.block<3, 3>(0, 0); //(0,0)处6x6块.H^T*T
    H_T_H.block<3, 3>(6, 6) = H_T_H_Sub.block<3, 3>(3, 3); //(0,0)处6x6块.H^T*T
    H_T_H.block<3, 3>(0, 6) = H_T_H_Sub.block<3, 3>(0, 3); //(0,0)处6x6块.H^T*T
    H_T_H.block<3, 3>(6, 0) = H_T_H_Sub.block<3, 3>(3, 0); //(0,0)处6x6块.H^T*T
    // std::cout << "ieskf_update 1 2" << std::endl;
    // 2>:求公式(20)中的Kalman增益的前面部分(省略R) : (H^T * R^-1 * H + P^-1)^-1

    Eigen::Matrix<double, 6, 1> HTz = HsubTCI * res_vec;

    Eigen::Matrix<double, 18, 1> HTCI; // n*1
    HTCI.setZero();
    HTCI.block<3, 1>(0, 0) = HTz.block<3, 1>(0, 0);
    HTCI.block<3, 1>(6, 0) = HTz.block<3, 1>(3, 0);
    // HTCI.block(0,0, 3,res_num) = HsubTCI.block(0,0, 3,res_num);
    // HTCI.block(6,0, 3,res_num) = HsubTCI.block(3,0, 3,res_num);
    // std::cout << "ieskf_update 1 2 1" << std::endl;

    K_1 = (H_T_H + temp_states_.back().cov_.inverse()).inverse();
    // std::cout << "ieskf_update 1 2 2" << std::endl;
    auto KH = K_1 * H_T_H;

    // 4>:求公式(18)中的最右边部分 : x^kk-x^k : Kalman迭代时的传播状态-预估(也是更新)状态
    auto vec = temp_states_.back() - system_state_; // state_propagate初始=g_lio_state
    // 5>:求公式(18)的中间和右边部分(有出入:I什么的都省略了)

    // Eigen::Matrix<double, 18, 1> dx = - (K_1 * HTCI) + vec - KH*vec; // 按此公式计算dx,当循环迭代次数越多，就会越不稳定， 将 vec - KH*vec 去掉后会变好。
    Eigen::Matrix<double, 18, 1> dx = -(K_1 * HTCI) - vec + KH * vec;

    temp_states_.back() += dx;
    if (flg_final_iter) {
        temp_states_.back().cov_ = (I_STATE - KH) * temp_states_.back().cov_;
        system_state_ = temp_states_.back();
        system_state_temp_for_smooth_ = system_state_;
    }
}

void Imu_Integration::smooth()
{
    temp_states_.back() = system_state_temp_for_smooth_;
    State_Group temp_state;

    double dt;
    Eigen::Vector3d un_acc_0, un_acc_1, un_acc, un_gyr;
    Eigen::Matrix<double, 18, 18> F; // p（3）、v（3）、q（3）、ba（3）、bg（3）
    Eigen::Matrix<double, 18, 12> V; // acc（3）、gyr（3）、ba（3）、bg（3）
    Eigen::Matrix<double, 18, 18> G;
    Eigen::Matrix<double, 18, 1> dx1, dx2;
    for (int idx = data_size_ - 2; idx >= 0; idx--) {
        // state_0 为要进行smooth的帧（当前帧）， state_1为下一帧，
        State_Group &state_0 = temp_states_[idx];
        State_Group &state_1 = temp_states_[idx + 1];

        // 根据当前帧状态预测下一帧状态
        dt = time_buf_[idx + 1] - time_buf_[idx];
        un_gyr = 0.5 * (gyr_buf_[idx] + gyr_buf_[idx + 1]) - state_0.bg_;
        temp_state.quat_ = state_0.quat_ * MathTools::Exp_Quat(un_gyr, dt) /* MathTools::Exp(un_gyr, dt) */;
        un_acc_0 = state_0.quat_ * (acc_buf_[idx] - state_0.ba_);
        un_acc_1 = state_1.quat_ * (acc_buf_[idx + 1] - state_0.ba_);
        un_acc = 0.5 * (un_acc_0 + un_acc_1);

        temp_state.pos_ = state_0.pos_ + state_0.vel_ * dt + 0.5 * (un_acc + state_0.grav_) * dt * dt;
        temp_state.vel_ = state_0.vel_ + (un_acc + state_0.grav_) * dt;
        temp_state.ba_ = state_0.ba_;
        temp_state.bg_ = state_0.bg_;
        temp_state.grav_ = state_0.grav_;

        // 根据当前状态协方差预测下一帧协方差
        F = Eigen::Matrix<double, 18, 18>::Identity();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(3, 6) = -state_0.quat_.toRotationMatrix() * MathTools::SkewSymmetric(un_acc) * dt;
        F.block<3, 3>(3, 9) = -state_0.quat_.toRotationMatrix() * dt;
        F.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - MathTools::SkewSymmetric(un_gyr) * dt;
        F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

        V = Eigen::Matrix<double, 18, 12>::Zero();
        V.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * dt;
        V.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;
        V.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity() * sqrt(dt);
        V.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * sqrt(dt);
        temp_state.cov_ = F * state_0.cov_ * F.transpose() + V * imu_noise_cov_ * V.transpose();

        G = state_0.cov_ * F.transpose() * temp_state.cov_.inverse();
        dx1.block<3, 1>(0, 0) = state_1.pos_ - temp_state.pos_;
        dx1.block<3, 1>(3, 0) = state_1.vel_ - temp_state.vel_;
        dx1.block<3, 1>(6, 0) = 2 * (temp_state.quat_.inverse() * state_1.quat_).vec();
        dx1.block<3, 1>(9, 0) = state_1.ba_ - temp_state.ba_;
        dx1.block<3, 1>(12, 0) = state_1.bg_ - temp_state.bg_;
        dx1.block<3, 1>(15, 0) = state_1.grav_ - temp_state.grav_;
        dx2 = G * dx1;

        state_0 += dx2;
    }
    system_state_temp_for_smooth_ = temp_states_.front();
}