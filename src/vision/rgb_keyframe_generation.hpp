#pragma once
#include <queue>
#include <cmath>

#include <Eigen/Core>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../basic_element/camera_intrinsic.hpp"

/**
 * @brief RGB关键帧子地图生成器。
 *
 * 该模块将连续累积的LiDAR点投影到当前图像平面，从图像中采样颜色并生成
 * 一帧彩色子地图。每次成功生成后，将结果放入队列供外部异步消费。
 *
 * 设计要点：
 * 1. 输入图像可能有不同通道格式，内部统一转换为BGR。
 * 2. 为避免图像延迟导致点云无限积累，提供子地图点数上限保护。
 * 3. 仅在深度范围内且投影落在图像边界内的点参与着色。
 */
template <typename PointType> class Rgb_KeyFrame_Generation {
public:
    /**
     * @brief 构造函数。
     * @param camera_intrinsic 相机内参对象指针，生命周期由外部管理。
     */
    explicit Rgb_KeyFrame_Generation(Camera_Intrinsic *camera_intrinsic)
        : rgb_submap_(new pcl::PointCloud<pcl::PointXYZRGB>())
        , submap_(new pcl::PointCloud<PointType>())
        , camera_intrinsic_(camera_intrinsic)
    {
    }

    /**
     * @brief 添加一帧LiDAR/图像信息，并在图像可用时生成彩色子地图。
     *
     * 执行流程：
     * 1. 累积LiDAR点到临时子地图。
     * 2. 图像可用时，将子地图点变换到相机坐标系并投影到像素平面。
     * 3. 读取像素颜色写入RGB点云，推入输出队列。
     * 4. 重置临时缓存，为下一关键帧做准备。
     */
    void add_frame(double timestamp, const Eigen::Matrix3d &pose_rot, const Eigen::Vector3d &pose_pos, typename pcl::PointCloud<PointType>::Ptr cloud, const cv::Mat &img)
    {
        if (!cloud || cloud->empty()) {
            return;
        }

        // Prevent unbounded accumulation when image updates are delayed.
        if (submap_->size() + cloud->size() > max_submap_points_) {
            submap_->clear();
            rgb_submap_->clear();
        }

        *submap_ += *cloud;

        if (!img.empty()) {
            cv::Mat bgr_img = ensure_bgr_image(img);
            if (bgr_img.empty() || !camera_intrinsic_) {
                submap_->clear();
                rgb_submap_->clear();
                return;
            }

            Eigen::Matrix3d pose_rot_inv = pose_rot.transpose();
            Eigen::Vector3d pose_pos_inv = -pose_rot_inv * pose_pos;
            pcl::PointXYZRGB pt_rgb_tmp;
            Eigen::Vector3d pt_tmp;
            double pixel_u = 0.0;
            double pixel_v = 0.0;
            const int img_cols = bgr_img.cols;
            const int img_rows = bgr_img.rows;
            cv::Mat overlay_img = bgr_img.clone();
            size_t proj_count = 0;

            // 遍历当前子地图，将每个点投影到图像平面并读取颜色。
            for (const auto &pt : submap_->points) {
                pt_tmp = pose_rot_inv * Eigen::Vector3d(pt.x, pt.y, pt.z) + pose_pos_inv;
                if (pt_tmp[2] > 0.1 && pt_tmp[2] < 50.0) {
                    camera_intrinsic_->camera_2_pixel(pt_tmp, pixel_u, pixel_v);
                    if (camera_intrinsic_->in_border(pixel_u, pixel_v)) {
                        const int pixel_u_i = static_cast<int>(pixel_u);
                        const int pixel_v_i = static_cast<int>(pixel_v);
                        if (pixel_u_i < 0 || pixel_u_i >= img_cols || pixel_v_i < 0 || pixel_v_i >= img_rows) {
                            continue;
                        }

                        const cv::Vec3b &pixel = bgr_img.at<cv::Vec3b>(pixel_v_i, pixel_u_i);
                        pt_rgb_tmp.x = pt.x;
                        pt_rgb_tmp.y = pt.y;
                        pt_rgb_tmp.z = pt.z;
                        pt_rgb_tmp.r = pixel[2];
                        pt_rgb_tmp.g = pixel[1];
                        pt_rgb_tmp.b = pixel[0];
                        rgb_submap_->push_back(pt_rgb_tmp);

                        // Draw sparse projected lidar points for direct image-lidar alignment visualization.
                        if ((proj_count % overlay_draw_stride_) == 0) {
                            cv::circle(overlay_img, cv::Point(pixel_u_i, pixel_v_i), overlay_point_radius_px_, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
                        }
                        proj_count++;
                    }
                }
            }

            rgb_submap_queue_.push(rgb_submap_);
            key_position_queue_.push(pose_pos);
            key_timestamp_queue_.push(timestamp);

            while (rgb_submap_queue_.size() > max_queue_size_) {
                rgb_submap_queue_.pop();
                key_position_queue_.pop();
                key_timestamp_queue_.pop();
            }

            overlay_image_queue_.push(overlay_img);
            overlay_timestamp_queue_.push(timestamp);
            while (overlay_image_queue_.size() > max_queue_size_) {
                overlay_image_queue_.pop();
                overlay_timestamp_queue_.pop();
            }

            rgb_submap_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
            submap_.reset(new pcl::PointCloud<PointType>());
        }
    }

    /**
     * @brief 取出最早生成的一帧RGB子地图。
     * @return true 队列非空并成功取出；false 队列为空。
     *
     * @note 当前实现仅输出位姿平移，四元数默认设为单位阵。
     */
    bool get_rgb_submap(double &timestamp, Eigen::Quaterniond &pose_quat, Eigen::Vector3d &pose_pos, typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr &rgb_submap)
    {
        if (rgb_submap_queue_.empty()) {
            return false;
        }

        timestamp = key_timestamp_queue_.front();
        key_timestamp_queue_.pop();
        pose_pos = key_position_queue_.front();
        key_position_queue_.pop();
        pose_quat = Eigen::Quaterniond::Identity();
        rgb_submap = rgb_submap_queue_.front();
        rgb_submap_queue_.pop();
        return true;
    }

    bool get_overlay_image(double &timestamp, cv::Mat &overlay_img)
    {
        if (overlay_image_queue_.empty()) {
            return false;
        }

        timestamp = overlay_timestamp_queue_.front();
        overlay_timestamp_queue_.pop();
        overlay_img = overlay_image_queue_.front();
        overlay_image_queue_.pop();
        return true;
    }

private:
    /**
     * @brief 将输入图像转换到BGR三通道格式。
     *
     * 支持输入类型：
     * - CV_8UC3: 直接返回。
     * - CV_8UC1: 灰度转BGR。
     * - CV_8UC4: BGRA转BGR。
     */
    cv::Mat ensure_bgr_image(const cv::Mat &img) const
    {
        if (img.empty()) {
            return cv::Mat();
        }

        if (img.type() == CV_8UC3) {
            return img;
        }

        cv::Mat bgr;
        if (img.type() == CV_8UC1) {
            cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);
            return bgr;
        }

        if (img.type() == CV_8UC4) {
            cv::cvtColor(img, bgr, cv::COLOR_BGRA2BGR);
            return bgr;
        }

        return cv::Mat();
    }

    // 当前关键帧对应的彩色点云与待着色点云缓存。
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_submap_;
    typename pcl::PointCloud<PointType>::Ptr submap_;

    // 关键帧输出队列（时间、位置、彩色子地图一一对应）。
    std::queue<typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr> rgb_submap_queue_;
    std::queue<Eigen::Vector3d> key_position_queue_;
    std::queue<double> key_timestamp_queue_;
    std::queue<cv::Mat> overlay_image_queue_;
    std::queue<double> overlay_timestamp_queue_;

    // 队列与临时子地图容量保护。
    const size_t max_queue_size_ = 10;
    const size_t max_submap_points_ = 1500000;
    const size_t overlay_draw_stride_ = 5;
    const int overlay_point_radius_px_ = 2; // Diameter 5px ~= 5x of a 1px point.

    // 相机内参配置。
    Camera_Intrinsic *camera_intrinsic_;
};
