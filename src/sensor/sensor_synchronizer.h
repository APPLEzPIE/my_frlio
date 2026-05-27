
#pragma once
#include <queue>
#include <deque>
#include <memory>
#include <mutex>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "./sensor_data.hpp"
#include "../tools/math_tools.hpp"

/**
 * @brief 一组时间同步后的LiDAR-IMU测量。
 */
template <typename PointType> struct Lidar_Imu_Measure_Group {
    Lidar_Imu_Measure_Group()
        : lidar_data_(nullptr)
        , lidar_begin_time_(0)
        , lidar_end_time_(0)
    {
        ;
    }

    double lidar_begin_time_;
    double lidar_end_time_;
    std::shared_ptr<Lidar_Data<PointType>> lidar_data_;
    std::deque<std::shared_ptr<Imu_Data>> imu_datas_;
};

template <typename PointType> class Sensor_Synchronizer {
public:
    /**
     * @brief 写入图像数据到同步缓存。
     *
     * 若检测到时间回拨，则清空图像缓存，避免跨时间段误配对。
     */
    void add_image_data(std::shared_ptr<Image_Data> image_data)
    {
        std::lock_guard<std::mutex> lock(mtx_sensor_);
        static double last_image_timestamp = -1;

        if (image_data->timestamp_ < last_image_timestamp) {
            ROS_WARN("image loop back, clear buffer");
            std::deque<std::shared_ptr<Image_Data>>().swap(image_data_buf_);
        }
        last_image_timestamp = image_data->timestamp_;

        image_data_buf_.push_back(image_data);
        while (image_data_buf_.size() > max_image_buffer_size_) {
            image_data_buf_.pop_front();
        }
    }

    /**
     * @brief 写入LiDAR帧数据到同步缓存。
     */
    void add_lidar_data(std::shared_ptr<Lidar_Data<PointType>> lidar_data)
    {
        std::lock_guard<std::mutex> lock(mtx_sensor_);
        lidar_data_buf_.push_back(lidar_data);
        while (lidar_data_buf_.size() > max_lidar_buffer_size_) {
            lidar_data_buf_.pop_front();
        }
    }

    /**
     * @brief 写入IMU数据到同步缓存。
     *
     * 若检测到时间回拨，则清空IMU缓存，避免时序污染。
     */
    void add_imu_data(std::shared_ptr<Imu_Data> imu_data)
    {
        std::lock_guard<std::mutex> lock(mtx_sensor_);
        static double last_imu_timestamp = -1;

        if (imu_data->timestamp_ < last_imu_timestamp) {
            ROS_WARN("imu loop back, clear buffer");
            std::deque<std::shared_ptr<Imu_Data>>().swap(imu_data_buf_);
        }
        last_imu_timestamp = imu_data->timestamp_;

        imu_data_buf_.push_back(imu_data);
        while (imu_data_buf_.size() > max_imu_buffer_size_) {
            imu_data_buf_.pop_front();
        }
    }

    /**
     * @brief LiDAR和IMU数据同步函数
     *
     * 该函数实现LiDAR和IMU数据的时间同步。它从各自的缓冲区中提取数据，
     * 确保一帧LiDAR数据与其时间范围内的所有IMU数据相关联。
     *
     * 同步策略：
     * 1. 确保存在可用的LiDAR数据
     * 2. 从缓冲区中取出一帧LiDAR数据（包含开始和结束时间戳）
     * 3. 对于第一帧LiDAR数据，确保队列中最早的IMU数据早于LiDAR开始时间
     * 4. 等待直到有IMU数据晚于LiDAR结束时间（确保有足够的IMU数据覆盖整个LiDAR帧）
     * 5. 收集时间范围内的IMU数据，并对边界处的IMU数据进行线性插值
     *
     * @param lidar_imu_syn_meas 输出参数，包含同步后的LiDAR和IMU数据的测量组
     *
     * @return true  成功同步一组LiDAR-IMU数据
     * @return false 同步条件未满足，需继续等待
     *
     * @note
     * - 使用互斥锁保护对共享缓冲区的访问
     * - 使用静态变量维护同步状态和临时IMU数据
     * - IMU数据在LiDAR帧的边界处使用线性插值生成
     * - 第一帧处理时会丢弃所有早于LiDAR开始时间的IMU数据
     *
     * @warning
     * - 依赖于时间戳的单调递增性
     * - 如果IMU缓冲区为空，可能导致未定义行为，需确保IMU数据持续供应
     */
    bool lidar_imu_sync(std::shared_ptr<Lidar_Imu_Measure_Group<PointType>> &lidar_imu_syn_meas)
    {
        static std::shared_ptr<Lidar_Imu_Measure_Group<PointType>> syn_measures = std::make_shared<Lidar_Imu_Measure_Group<PointType>>();
        static std::shared_ptr<Lidar_Imu_Measure_Group<PointType>> last_syn_measures = std::make_shared<Lidar_Imu_Measure_Group<PointType>>();
        static std::shared_ptr<Imu_Data> tmp_imu_data;
        std::lock_guard<std::mutex> lock(mtx_sensor_);

        // LOG(INFO) << "lidar_data_buf_.size():" << lidar_data_buf_.size() << " ,imu_data_buf_.size():" << imu_data_buf_.size() << std::endl;

        // 保证当前有可用的LiDAR数据，否则继续等待
        if (!syn_measures->lidar_data_ && lidar_data_buf_.empty()) {
            return false;
        }

        // 取出一帧LiDAR数据
        if (!syn_measures->lidar_data_) {
            syn_measures->lidar_data_ = lidar_data_buf_.front();
            lidar_data_buf_.pop_front();
            syn_measures->lidar_begin_time_ = syn_measures->lidar_data_->begin_time_;
            syn_measures->lidar_end_time_ = syn_measures->lidar_data_->end_time_;
        }

        // static double last_lidar_time = syn_measures->lidar_begin_time_ - 0.1;
        // if(syn_measures->lidar_begin_time_ - last_lidar_time > 0.11 || syn_measures->lidar_begin_time_ - last_lidar_time < 0.09) {
        //     std::cout<<"curr_lidar_begin_time - last_lidar_begin_time = " << syn_measures->lidar_begin_time_ - last_lidar_time<<std::endl;
        //     exit(0);
        // }
        // last_lidar_time = syn_measures->lidar_begin_time_;

        // 对于第一个LiDAR帧，保证队列中第一个IMU数据比 lidar_begin_time_ 旧，否则丢弃当前LiDAR帧
        if (!imu_data_buf_.empty() && (tmp_imu_data.use_count() == 0) && (imu_data_buf_.front()->timestamp_ > syn_measures->lidar_begin_time_)) {
            syn_measures->lidar_data_ = nullptr;
            return false;
        }

        // 保证队列中最后一个IMU数据比 lidar_end_time_ 新，否则继续等待IMU数据
        if (!imu_data_buf_.empty() && imu_data_buf_.back()->timestamp_ > syn_measures->lidar_end_time_) {
            syn_measures->imu_datas_.clear();

            // 对于第一帧，首先将过于旧的IMU数据全部丢弃。
            if (tmp_imu_data.use_count() == 0) {
                while ((!imu_data_buf_.empty()) && (imu_data_buf_.front()->timestamp_ < syn_measures->lidar_begin_time_)) {
                    tmp_imu_data = imu_data_buf_.front();
                    imu_data_buf_.pop_front();
                }
                last_syn_measures->lidar_end_time_ = syn_measures->lidar_begin_time_;
            }

            // 插值获取第一帧IMU数据
            {
                double time = last_syn_measures->lidar_end_time_;
                double dt = imu_data_buf_.front()->timestamp_ - tmp_imu_data->timestamp_;
                Eigen::Vector3d acc;
                Eigen::Vector3d gyr;
                if (std::fabs(dt) < 1e-9) {
                    acc = tmp_imu_data->acc_;
                    gyr = tmp_imu_data->gyr_;
                } else {
                    double ratio_next = (time - tmp_imu_data->timestamp_) / dt;
                    double ratio_prev = (imu_data_buf_.front()->timestamp_ - time) / dt;
                    acc = imu_data_buf_.front()->acc_ * ratio_next + tmp_imu_data->acc_ * ratio_prev;
                    gyr = imu_data_buf_.front()->gyr_ * ratio_next + tmp_imu_data->gyr_ * ratio_prev;
                }
                syn_measures->imu_datas_.emplace_back(new Imu_Data(time, acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]));
            }

            // 获取中间的IMU数据数据
            while ((!imu_data_buf_.empty()) && (imu_data_buf_.front()->timestamp_ < syn_measures->lidar_end_time_)) {
                syn_measures->imu_datas_.push_back(imu_data_buf_.front());
                tmp_imu_data = imu_data_buf_.front();
                imu_data_buf_.pop_front();
            }

            // 插值获取最后一帧IMU数据
            {
                double time = syn_measures->lidar_end_time_;
                double dt = imu_data_buf_.front()->timestamp_ - tmp_imu_data->timestamp_;
                Eigen::Vector3d acc;
                Eigen::Vector3d gyr;
                if (std::fabs(dt) < 1e-9) {
                    acc = tmp_imu_data->acc_;
                    gyr = tmp_imu_data->gyr_;
                } else {
                    double ratio_next = (time - tmp_imu_data->timestamp_) / dt;
                    double ratio_prev = (imu_data_buf_.front()->timestamp_ - time) / dt;
                    acc = imu_data_buf_.front()->acc_ * ratio_next + tmp_imu_data->acc_ * ratio_prev;
                    gyr = imu_data_buf_.front()->gyr_ * ratio_next + tmp_imu_data->gyr_ * ratio_prev;
                }
                syn_measures->imu_datas_.emplace_back(new Imu_Data(time, acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]));
            }

            lidar_imu_syn_meas = syn_measures;
            last_syn_measures = syn_measures;
            syn_measures.reset(new Lidar_Imu_Measure_Group<PointType>());
            return true;
        }
        return false;
    }

    void print_syn_meas(std::shared_ptr<Lidar_Imu_Measure_Group<PointType>> lidar_image_imu_syn_meas)
    {
        std::cout << std::fixed << std::setprecision(9) << "[a syn meas]: " << std::endl
                  << "[lidar]" << std::endl
                  << "    [begin time]: " << lidar_image_imu_syn_meas->lidar_begin_time_ << std::endl
                  << "    [end   time]: " << lidar_image_imu_syn_meas->lidar_end_time_ << std::endl
                  << "[imu]: " << std::endl;
        for (auto imu_data : lidar_image_imu_syn_meas->imu_datas_) {
            std::cout << "      [imu frame]: " << imu_data->timestamp_ << " acc:" << imu_data->acc_.transpose() << " gyr:" << imu_data->gyr_.transpose() << std::endl;
        }
    }

    /**
     * @brief 获取指定时间戳之前最近的一帧图像。
     *
     * 队列中早于timestamp的图像会被消费，并保留最近一帧作为返回值。
     */
    std::shared_ptr<Image_Data> get_latest_image_before(double timestamp)
    {
        std::lock_guard<std::mutex> lock(mtx_sensor_);
        while (!image_data_buf_.empty() && image_data_buf_.front()->timestamp_ <= timestamp) {
            latest_image_data_ = image_data_buf_.front();
            image_data_buf_.pop_front();
        }
        return latest_image_data_;
    }

private:
    // 统一使用同一把锁保护多传感器缓存并发访问。
    std::mutex mtx_sensor_;

    // 原始传感器缓存。
    std::deque<std::shared_ptr<Image_Data>> image_data_buf_;
    std::deque<std::shared_ptr<Lidar_Data<PointType>>> lidar_data_buf_;
    std::deque<std::shared_ptr<Imu_Data>> imu_data_buf_;

    // 最近一次有效图像缓存，供按时间查询接口复用。
    std::shared_ptr<Image_Data> latest_image_data_;

    // 缓存上限用于防止在输入堆积时内存无界增长。
    const size_t max_image_buffer_size_ = 2000;
    const size_t max_lidar_buffer_size_ = 40;
    const size_t max_imu_buffer_size_ = 20000;
};