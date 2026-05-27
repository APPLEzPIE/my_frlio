#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "../basic_element/state_group.hpp"
#include "../sensor/sensor_data.hpp"
#include "../tools/math_tools.hpp"

/**
 * @brief LiDAR运动补偿工具。
 *
 * 基于高频位姿序列对扫描周期内点云进行去畸变，
 * 输出统一时刻下的无畸变点云帧。
 */
class Lidar_Motion_Compensate_Tool {
public:
    template <typename PointType> struct Undis_LiDAR_Frame {
        double timestamp_;
        Eigen::Quaterniond pose_quat_;
        Eigen::Vector3d pose_pos_;
        typename pcl::PointCloud<PointType>::Ptr cloud_;
    };

    Eigen::Quaterniond ext_quat_i2l_;
    Eigen::Vector3d ext_pos_i2l_;
    std::vector<double> v_timestamp_;
    std::vector<Eigen::Quaterniond> v_pose_quat_m2l_;
    std::vector<Eigen::Vector3d> v_pose_pos_m2l_;
    Lidar_Motion_Compensate_Tool(const Eigen::Quaterniond &ext_quat_i2l = Eigen::Quaterniond::Identity(), const Eigen::Vector3d &ext_pos_i2l = Eigen::Vector3d::Zero())
        : ext_quat_i2l_(ext_quat_i2l)
        , ext_pos_i2l_(ext_pos_i2l)
    {
    }

    Lidar_Motion_Compensate_Tool(const std::vector<State_Group> &poses, const Eigen::Quaterniond &ext_quat_i2l = Eigen::Quaterniond::Identity(),
                                 const Eigen::Vector3d &ext_pos_i2l = Eigen::Vector3d::Zero())
        : ext_quat_i2l_(ext_quat_i2l)
        , ext_pos_i2l_(ext_pos_i2l)
    {
        add_high_freq_pose(poses);
    }

    void add_high_freq_pose(const State_Group &pose)
    {
        v_timestamp_.emplace_back(pose.timestamp_);
        v_pose_quat_m2l_.emplace_back(pose.quat_ * ext_quat_i2l_);
        v_pose_pos_m2l_.emplace_back(pose.quat_ * ext_pos_i2l_ + pose.pos_);
    }

    void add_high_freq_pose(const std::vector<State_Group> &poses)
    {
        for (auto &pose : poses) {
            v_timestamp_.emplace_back(pose.timestamp_);
            v_pose_quat_m2l_.emplace_back(pose.quat_ * ext_quat_i2l_);
            v_pose_pos_m2l_.emplace_back(pose.quat_ * ext_pos_i2l_ + pose.pos_);
        }
    }

    // 输入原始点云、投影时刻及对应的位姿， 返回投影到 投影时刻 的点云
    template <typename PointType>
    typename pcl::PointCloud<PointType>::Ptr motion_compensation_for_one_frame(typename pcl::PointCloud<PointType>::Ptr distorted_cloud, const Eigen::Quaterniond &proj_quat_m2i,
                                                                               const Eigen::Vector3d &proj_pos_m2i)
    {
        // 创建输出点云：用于存储投影到统一时刻后的无畸变点。
        typename pcl::PointCloud<PointType>::Ptr local_undistorted_cloud(new pcl::PointCloud<PointType>());

        // 将“投影时刻下的 IMU->世界”姿态结合外参，得到“投影时刻下 LiDAR->世界”的逆，即 世界->LiDAR（投影时刻）。
        // 这里记作 proj_quat_l2m，后续用于把世界系点转回投影时刻的 LiDAR 系。
        Eigen::Quaterniond proj_quat_l2m = (proj_quat_m2i * ext_quat_i2l_).inverse();

        // 计算投影时刻下世界原点在 LiDAR 坐标系中的平移项。
        // 与上面的旋转项配套，构成 world -> lidar(proj_time) 的完整刚体变换。
        Eigen::Vector3d proj_pos_l2m = ext_quat_i2l_.inverse() * (proj_quat_m2i.inverse() * (-proj_pos_m2i) - ext_pos_i2l_);

        // 临时点：保存当前处理后的单点结果。
        PointType undistorted_pt;

        // pt_eigen：当前点的三维向量形式。
        // pos_m2l / quat_m2l：当前点时间戳对应的世界->LiDAR位姿（通过高频位姿插值得到）。
        Eigen::Vector3d pt_eigen, pos_m2l;
        Eigen::Quaterniond quat_m2l;

        // 当前点对应的高频位姿区间左端索引。
        int high_freq_pose_idx;
#ifdef MY_DEBUG_MODE
        // 调试保护：至少需要两帧位姿才能进行时间插值。
        if (v_timestamp_.size() < 2) {
            LOG(FATAL) << "v_timestamp_.size() < 2" << std::endl;
        }
#endif

        // 情况1：位姿数量大于3，使用“近似等间隔索引 + 邻近两帧精确插值”的主流程。
        if (v_timestamp_.size() > 3) {
            // 位姿总数。
            int pose_num = v_timestamp_.size();
            // 最大可作为左端点的索引（右端要访问 idx+1）。
            int pose_num_minus_2 = pose_num - 2;
            // 从第2个时间戳开始用于快速定位（与原实现保持一致）。
            double high_freq_pose_time_begin = v_timestamp_[1];
            // 近似平均时间间隔，用于把点时间快速映射到候选索引。
            double average_dt = (v_timestamp_[pose_num_minus_2] - v_timestamp_[1]) / (pose_num - 3);

            // 逐点去畸变。
            for (auto &distorted_pt : distorted_cloud->points) {
                // 负时间戳通常表示无效点，直接跳过。
                if (distorted_pt.timestamp < 0) {
                    continue;
                }

                // 若点时间在快速定位起始时间之后，按平均间隔估算其所在位姿索引。
                if (distorted_pt.timestamp > high_freq_pose_time_begin) {
                    high_freq_pose_idx = (distorted_pt.timestamp - high_freq_pose_time_begin) / average_dt + 1;
                    // 索引钳制，避免越界到最后一帧之后。
                    if (high_freq_pose_idx > pose_num_minus_2) {
                        high_freq_pose_idx = pose_num_minus_2;
                    }
                } else {
                    // 起始时间之前统一使用第0帧作为左端点。
                    high_freq_pose_idx = 0;
                }

                // 旋转插值：在 [idx, idx+1] 两帧之间按时间比例进行球面线性插值（slerp）。
                quat_m2l = v_pose_quat_m2l_[high_freq_pose_idx].slerp(
                    ((distorted_pt.timestamp - v_timestamp_[high_freq_pose_idx]) / (v_timestamp_[high_freq_pose_idx + 1] - v_timestamp_[high_freq_pose_idx])),
                    v_pose_quat_m2l_[high_freq_pose_idx + 1]);

                // 平移插值：与旋转同时间比例的线性插值。
                pos_m2l =
                    v_pose_pos_m2l_[high_freq_pose_idx] *
                        ((v_timestamp_[high_freq_pose_idx + 1] - distorted_pt.timestamp) / (v_timestamp_[high_freq_pose_idx + 1] - v_timestamp_[high_freq_pose_idx])) +
                    v_pose_pos_m2l_[high_freq_pose_idx + 1] * ((distorted_pt.timestamp - v_timestamp_[high_freq_pose_idx]) / (v_timestamp_[high_freq_pose_idx + 1] - v_timestamp_[high_freq_pose_idx]));

                // 读取原始点坐标。
                pt_eigen << distorted_pt.x, distorted_pt.y, distorted_pt.z;

                // 坐标变换链：
                // 1) 先用当前点时刻的 m2l 位姿，把点从当前 LiDAR 坐标映射到世界系；
                // 2) 再用投影时刻的 l2m 逆变换，把世界系点映射到“投影时刻 LiDAR 系”。
                pt_eigen = proj_quat_l2m * (quat_m2l * pt_eigen + pos_m2l) + proj_pos_l2m;

                // 写回到输出点。
                undistorted_pt.x = pt_eigen[0];
                undistorted_pt.y = pt_eigen[1];
                undistorted_pt.z = pt_eigen[2];

                // 追加到当前帧的无畸变点云。
                local_undistorted_cloud->push_back(undistorted_pt);

#ifdef MY_DEBUG_MODE
                // 若变换后坐标异常大，输出调试信息定位问题。
                if (pt_eigen.norm() > 1000) {
                    LOG(ERROR) << "pt transformed: " << pt_eigen.transpose() << std::endl
                               << "pt origin: " << distorted_pt.x << ", " << distorted_pt.y << ", " << distorted_pt.z << std::endl
                               << "pos_m2l: " << pos_m2l.transpose() << ", " << quat_m2l.coeffs().transpose() << std::endl
                               << "v_timestamp_ size: " << v_timestamp_.size() << ", first time: " << v_timestamp_.front() << ", end time: " << v_timestamp_.back() << std::endl
                               << "point time: " << distorted_pt.timestamp << ", high_freq_pose_idx:" << high_freq_pose_idx << std::endl;
                }
#endif
            }

            // 情况2：恰好3帧位姿，分成两个时间区间分别插值。
        } else if (v_timestamp_.size() == 3) {
            // 三个时间戳及两个区间长度。
            double first_timestamp = v_timestamp_[0], second_timestamp = v_timestamp_[1], third_timestamp = v_timestamp_[2], time_interval1, time_interval2;

            // 三帧姿态与位置缓存为局部变量，减少重复索引开销并提高可读性。
            Eigen::Quaterniond first_pose_quat = v_pose_quat_m2l_[0], second_pose_quat = v_pose_quat_m2l_[1], third_pose_quat = v_pose_quat_m2l_[2];
            Eigen::Vector3d first_pose_pos = v_pose_pos_m2l_[0], second_pose_pos = v_pose_pos_m2l_[1], third_pose_pos = v_pose_pos_m2l_[2];

            // 计算两个时间段的时长。
            time_interval1 = second_timestamp - first_timestamp;
            time_interval2 = third_timestamp - second_timestamp;

            // 逐点去畸变。
            for (auto &distorted_pt : distorted_cloud->points) {
                // 跳过无效时间戳点。
                if (distorted_pt.timestamp < 0) {
                    continue;
                }

                // 点在前半段 [t0, t1) 内：使用第1段插值。
                if (distorted_pt.timestamp < second_timestamp) {
                    quat_m2l = first_pose_quat.slerp(((distorted_pt.timestamp - first_timestamp) / time_interval1), second_pose_quat);
                    pos_m2l = first_pose_pos * ((second_timestamp - distorted_pt.timestamp) / time_interval1) + second_pose_pos * ((distorted_pt.timestamp - first_timestamp) / time_interval1);
                } else {
                    // 点在后半段 [t1, t2] 内：使用第2段插值。
                    quat_m2l = second_pose_quat.slerp(((distorted_pt.timestamp - second_timestamp) / time_interval2), third_pose_quat);
                    pos_m2l = second_pose_pos * ((third_timestamp - distorted_pt.timestamp) / time_interval2) + third_pose_pos * ((distorted_pt.timestamp - second_timestamp) / time_interval2);
                }

                // 点坐标读入与坐标系变换（同上主流程）。
                pt_eigen << distorted_pt.x, distorted_pt.y, distorted_pt.z;
                pt_eigen = proj_quat_l2m * (quat_m2l * pt_eigen + pos_m2l) + proj_pos_l2m;

                // 写入输出点云。
                undistorted_pt.x = pt_eigen[0];
                undistorted_pt.y = pt_eigen[1];
                undistorted_pt.z = pt_eigen[2];
                local_undistorted_cloud->push_back(undistorted_pt);

#ifdef MY_DEBUG_MODE
                // 调试输出：监控异常大的变换结果。
                if (pt_eigen.norm() > 1000) {
                    LOG(ERROR) << "pt transformed: " << pt_eigen.transpose() << std::endl
                               << "pt origin: " << distorted_pt.x << ", " << distorted_pt.y << ", " << distorted_pt.z << std::endl
                               << "pos_m2l: " << pos_m2l.transpose() << ", " << quat_m2l.coeffs().transpose() << std::endl
                               << "v_timestamp_ size: " << v_timestamp_.size() << ", first time: " << v_timestamp_.front() << ", end time: " << v_timestamp_.back() << std::endl
                               << "point time: " << distorted_pt.timestamp << ", high_freq_pose_idx:" << high_freq_pose_idx << std::endl;
                }
#endif
            }

            // 情况3：只有2帧位姿，整段时间直接在首末两帧之间插值。
        } else if (v_timestamp_.size() == 2) {
            // 两个时间戳与总时长。
            double first_timestamp = v_timestamp_[0], second_timestamp = v_timestamp_[1], time_interval;

            // 两帧姿态与位置。
            Eigen::Quaterniond first_pose_quat = v_pose_quat_m2l_[0], second_pose_quat = v_pose_quat_m2l_[1];
            Eigen::Vector3d first_pose_pos = v_pose_pos_m2l_[0], second_pose_pos = v_pose_pos_m2l_[1];

            // 时间跨度。
            time_interval = second_timestamp - first_timestamp;

            // 逐点去畸变。
            for (auto &distorted_pt : distorted_cloud->points) {
                // 跳过无效点。
                if (distorted_pt.timestamp < 0) {
                    continue;
                }

                // 在两帧之间按时间比例插值旋转和平移。
                quat_m2l = first_pose_quat.slerp(((distorted_pt.timestamp - first_timestamp) / time_interval), second_pose_quat);
                pos_m2l = first_pose_pos * ((second_timestamp - distorted_pt.timestamp) / time_interval) + second_pose_pos * ((distorted_pt.timestamp - first_timestamp) / time_interval);

                // 坐标变换到投影时刻 LiDAR 系。
                pt_eigen << distorted_pt.x, distorted_pt.y, distorted_pt.z;
                pt_eigen = proj_quat_l2m * (quat_m2l * pt_eigen + pos_m2l) + proj_pos_l2m;

                // 写回并加入输出点云。
                undistorted_pt.x = pt_eigen[0];
                undistorted_pt.y = pt_eigen[1];
                undistorted_pt.z = pt_eigen[2];
                local_undistorted_cloud->push_back(undistorted_pt);

#ifdef MY_DEBUG_MODE
                // 调试输出：检查异常结果。
                if (pt_eigen.norm() > 1000) {
                    LOG(ERROR) << "pt transformed: " << pt_eigen.transpose() << std::endl
                               << "pt origin: " << distorted_pt.x << ", " << distorted_pt.y << ", " << distorted_pt.z << std::endl
                               << "pos_m2l: " << pos_m2l.transpose() << ", " << quat_m2l.coeffs().transpose() << std::endl
                               << "v_timestamp_ size: " << v_timestamp_.size() << ", first time: " << v_timestamp_.front() << ", end time: " << v_timestamp_.back() << std::endl
                               << "point time: " << distorted_pt.timestamp << ", high_freq_pose_idx:" << high_freq_pose_idx << std::endl;
                }
#endif
            }
        }

        // 返回当前帧去畸变后的点云。
        return local_undistorted_cloud;
    }

    // 输入原始点云、投影时刻及对应的位姿， 返回投影到 投影时刻 的点云
    template <typename PointType>
    void motion_compensation_for_one_frame_proj_to_map(typename pcl::PointCloud<PointType>::Ptr distorted_cloud, typename pcl::PointCloud<PointType>::Ptr &global_undistorted_cloud)
    {
        if (global_undistorted_cloud == nullptr) {
            global_undistorted_cloud.reset(new pcl::PointCloud<PointType>());
        }

        PointType undistorted_pt;
        Eigen::Vector3d pt_eigen, pos_m2l;
        Eigen::Quaterniond quat_m2l;
        int high_freq_pose_idx;
#ifdef MY_DEBUG_MODE
        if (v_timestamp_.size() < 2) {
            LOG(FATAL) << "v_timestamp_.size() < 2" << std::endl;
        }
#endif
        if (v_timestamp_.size() > 3) {
            int pose_num = v_timestamp_.size();
            int pose_num_minus_2 = pose_num - 2;
            double high_freq_pose_time_begin = v_timestamp_[1];
            double average_dt = (v_timestamp_[pose_num_minus_2] - v_timestamp_[1]) / (pose_num - 3);
            for (auto &distorted_pt : distorted_cloud->points) {
                if (distorted_pt.timestamp < 0) {
                    continue;
                }
                if (distorted_pt.timestamp > high_freq_pose_time_begin) {
                    high_freq_pose_idx = (distorted_pt.timestamp - high_freq_pose_time_begin) / average_dt + 1;
                    if (high_freq_pose_idx > pose_num_minus_2) {
                        high_freq_pose_idx = pose_num_minus_2;
                    }
                    // #ifdef MY_DEBUG_MODE
                    //     if(high_freq_pose_idx > pose_num_minus_2) {
                    //         LOG(FATAL) << "high_freq_pose_idx:" << high_freq_pose_idx << " ,pose_num_minus_2" << pose_num_minus_2 << std::endl;
                    //     }
                    // #endif
                } else {
                    high_freq_pose_idx = 0;
                }

                quat_m2l = v_pose_quat_m2l_[high_freq_pose_idx].slerp(
                    ((distorted_pt.timestamp - v_timestamp_[high_freq_pose_idx]) / (v_timestamp_[high_freq_pose_idx + 1] - v_timestamp_[high_freq_pose_idx])),
                    v_pose_quat_m2l_[high_freq_pose_idx + 1]);
                pos_m2l =
                    v_pose_pos_m2l_[high_freq_pose_idx] *
                        ((v_timestamp_[high_freq_pose_idx + 1] - distorted_pt.timestamp) / (v_timestamp_[high_freq_pose_idx + 1] - v_timestamp_[high_freq_pose_idx])) +
                    v_pose_pos_m2l_[high_freq_pose_idx + 1] * ((distorted_pt.timestamp - v_timestamp_[high_freq_pose_idx]) / (v_timestamp_[high_freq_pose_idx + 1] - v_timestamp_[high_freq_pose_idx]));

                pt_eigen << distorted_pt.x, distorted_pt.y, distorted_pt.z;
                pt_eigen = quat_m2l * pt_eigen + pos_m2l;
                undistorted_pt.x = pt_eigen[0];
                undistorted_pt.y = pt_eigen[1];
                undistorted_pt.z = pt_eigen[2];
                global_undistorted_cloud->push_back(undistorted_pt);

#ifdef MY_DEBUG_MODE
                if (pt_eigen.norm() > 1000000) {
                    LOG(ERROR) << "pt transformed: " << pt_eigen.transpose() << std::endl
                               << "pt origin: " << distorted_pt.x << ", " << distorted_pt.y << ", " << distorted_pt.z << std::endl
                               << "pos_m2l: " << pos_m2l.transpose() << ", " << quat_m2l.coeffs().transpose() << std::endl
                               << "v_timestamp_ size: " << v_timestamp_.size() << ", first time: " << v_timestamp_.front() << ", end time: " << v_timestamp_.back() << std::endl
                               << "point time: " << distorted_pt.timestamp << ", high_freq_pose_idx:" << high_freq_pose_idx << std::endl;
                }
#endif
            }
        } else if (v_timestamp_.size() == 3) {
            double first_timestamp = v_timestamp_[0], second_timestamp = v_timestamp_[1], third_timestamp = v_timestamp_[2], time_interval1, time_interval2;
            Eigen::Quaterniond first_pose_quat = v_pose_quat_m2l_[0], second_pose_quat = v_pose_quat_m2l_[1], third_pose_quat = v_pose_quat_m2l_[2];
            Eigen::Vector3d first_pose_pos = v_pose_pos_m2l_[0], second_pose_pos = v_pose_pos_m2l_[1], third_pose_pos = v_pose_pos_m2l_[2];
            time_interval1 = second_timestamp - first_timestamp;
            time_interval2 = third_timestamp - second_timestamp;
            for (auto &distorted_pt : distorted_cloud->points) {
                if (distorted_pt.timestamp < 0) {
                    continue;
                }
                if (distorted_pt.timestamp < second_timestamp) {
                    quat_m2l = first_pose_quat.slerp(((distorted_pt.timestamp - first_timestamp) / time_interval1), second_pose_quat);
                    pos_m2l = first_pose_pos * ((second_timestamp - distorted_pt.timestamp) / time_interval1) + second_pose_pos * ((distorted_pt.timestamp - first_timestamp) / time_interval1);
                } else {
                    quat_m2l = second_pose_quat.slerp(((distorted_pt.timestamp - second_timestamp) / time_interval2), third_pose_quat);
                    pos_m2l = second_pose_pos * ((third_timestamp - distorted_pt.timestamp) / time_interval2) + third_pose_pos * ((distorted_pt.timestamp - second_timestamp) / time_interval2);
                }

                pt_eigen << distorted_pt.x, distorted_pt.y, distorted_pt.z;
                pt_eigen = quat_m2l * pt_eigen + pos_m2l;
                undistorted_pt.x = pt_eigen[0];
                undistorted_pt.y = pt_eigen[1];
                undistorted_pt.z = pt_eigen[2];
                global_undistorted_cloud->push_back(undistorted_pt);

#ifdef MY_DEBUG_MODE
                if (pt_eigen.norm() > 1000000) {
                    LOG(ERROR) << "pt transformed: " << pt_eigen.transpose() << std::endl
                               << "pt origin: " << distorted_pt.x << ", " << distorted_pt.y << ", " << distorted_pt.z << std::endl
                               << "pos_m2l: " << pos_m2l.transpose() << ", " << quat_m2l.coeffs().transpose() << std::endl
                               << "v_timestamp_ size: " << v_timestamp_.size() << ", first time: " << v_timestamp_.front() << ", end time: " << v_timestamp_.back() << std::endl
                               << "point time: " << distorted_pt.timestamp << ", high_freq_pose_idx:" << high_freq_pose_idx << std::endl;
                }
#endif
            }
        } else if (v_timestamp_.size() == 2) {
            double first_timestamp = v_timestamp_[0], second_timestamp = v_timestamp_[1], time_interval;
            Eigen::Quaterniond first_pose_quat = v_pose_quat_m2l_[0], second_pose_quat = v_pose_quat_m2l_[1];
            Eigen::Vector3d first_pose_pos = v_pose_pos_m2l_[0], second_pose_pos = v_pose_pos_m2l_[1];
            time_interval = second_timestamp - first_timestamp;
            for (auto &distorted_pt : distorted_cloud->points) {
                if (distorted_pt.timestamp < 0) {
                    continue;
                }
                quat_m2l = first_pose_quat.slerp(((distorted_pt.timestamp - first_timestamp) / time_interval), second_pose_quat);
                pos_m2l = first_pose_pos * ((second_timestamp - distorted_pt.timestamp) / time_interval) + second_pose_pos * ((distorted_pt.timestamp - first_timestamp) / time_interval);

                pt_eigen << distorted_pt.x, distorted_pt.y, distorted_pt.z;
                pt_eigen = quat_m2l * pt_eigen + pos_m2l;
                undistorted_pt.x = pt_eigen[0];
                undistorted_pt.y = pt_eigen[1];
                undistorted_pt.z = pt_eigen[2];
                global_undistorted_cloud->push_back(undistorted_pt);

#ifdef MY_DEBUG_MODE
                if (pt_eigen.norm() > 1000000) {
                    LOG(ERROR) << "pt transformed: " << pt_eigen.transpose() << std::endl
                               << "pt origin: " << distorted_pt.x << ", " << distorted_pt.y << ", " << distorted_pt.z << std::endl
                               << "pos_m2l: " << pos_m2l.transpose() << ", " << quat_m2l.coeffs().transpose() << std::endl
                               << "v_timestamp_ size: " << v_timestamp_.size() << ", first time: " << v_timestamp_.front() << ", end time: " << v_timestamp_.back() << std::endl
                               << "point time: " << distorted_pt.timestamp << ", high_freq_pose_idx:" << high_freq_pose_idx << std::endl;
                }
#endif
            }
        }
    }

private:
    std::deque<State_Group> stamp_high_freq_poses_;
};