#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

/**
 * @brief 图像预处理模块。
 *
 * 支持分辨率调整、去畸变、灰度转换、直方图均衡等处理，
 * 并确保处理后内参与图像尺寸的一致性。
 */
// 用于处理输入的图像
// 提供 图层尺寸变换、去畸变、彩色转灰度、直方图均匀化等操作（隔离ros，只依赖opencv库）
// 需要的参数：所需要图像的大小、图像内参、畸变系数、是否进行尺寸变换、是否彩色转灰度、是否直方图均匀化、是否去畸变
// 需要假设接下来处理的所有图像大小都是相同的
// 自动根据读入的参数以及第一帧图像 获取处理后图像的 尺寸，内参。
class ImageFormatProcessor {
public:
    ImageFormatProcessor()
        : flg_first_img_(true)
    {
    }

    ImageFormatProcessor(bool param_flg_need_resize, bool param_flg_need_undistort, bool param_flg_need_equalize, int param_flg_rgb_or_bgr_or_gray, Eigen::Matrix3d cam_intrinsic_eigen,
                         Eigen::Matrix<double, 5, 1> cam_dist_coeffs_eigen, int resize_cols, int resize_rows)
        : flg_first_img_(true)
    {
        set_parameter_manually(param_flg_need_resize, param_flg_need_undistort, param_flg_need_equalize, param_flg_rgb_or_bgr_or_gray, cam_intrinsic_eigen, cam_dist_coeffs_eigen, resize_cols,
                               resize_rows);
    }

    void load_config_parameter(const std::string &sensor_config_file)
    {
        YAML::Node config_node = YAML::LoadFile(sensor_config_file);

        bool param_flg_need_resize;
        bool param_flg_need_undistort;
        bool param_flg_need_equalize;
        int param_flg_rgb_or_bgr_or_gray;
        Eigen::Matrix3d cam_intrinsic_eigen;
        Eigen::Matrix<double, 5, 1> cam_dist_coeffs_eigen;
        int resize_cols;
        int resize_rows;

        try {
            param_flg_need_resize = config_node["flg_cam_need_resize"].as<bool>();
            param_flg_need_undistort = config_node["flg_cam_need_undistort"].as<bool>();
            param_flg_need_equalize = config_node["flg_cam_need_equalize"].as<bool>();
            param_flg_rgb_or_bgr_or_gray = config_node["flg_cam_rgb_or_bgr_or_gray"].as<int>();
        } catch (...) {
            LOG(FATAL) << "[ImageFormatProcessor]: file " << sensor_config_file << " suffers from missing parameters.";
        }

        try {
            std::vector<double> cam_intrinsic_mat{ 9, 0.0 };
            cam_intrinsic_mat = config_node["cam_intrinsic_mat"].as<std::vector<double>>();
            cam_intrinsic_eigen << cam_intrinsic_mat[0], cam_intrinsic_mat[1], cam_intrinsic_mat[2], cam_intrinsic_mat[3], cam_intrinsic_mat[4], cam_intrinsic_mat[5], cam_intrinsic_mat[6],
                cam_intrinsic_mat[7], cam_intrinsic_mat[8];
        } catch (...) {
            try {
                double cam_intrinsic_fx, cam_intrinsic_fy, cam_intrinsic_cx, cam_intrinsic_cy;
                cam_intrinsic_fx = config_node["cam_intrinsic_fx"].as<double>();
                cam_intrinsic_fy = config_node["cam_intrinsic_fy"].as<double>();
                cam_intrinsic_cx = config_node["cam_intrinsic_cx"].as<double>();
                cam_intrinsic_cy = config_node["cam_intrinsic_cy"].as<double>();
                cam_intrinsic_eigen << cam_intrinsic_fx, 0, cam_intrinsic_cx, 0, cam_intrinsic_fy, cam_intrinsic_cy, 0, 0, 1;
            } catch (...) {
                LOG(FATAL) << "[ImageFormatProcessor]: file " << sensor_config_file << " has no camera intrinsic.";
            }
        }

        if (param_flg_need_undistort) {
            try {
                std::vector<double> cam_dist_coeffs_vec{ 5, 0.0 };
                cam_dist_coeffs_vec = config_node["cam_dist_coeffs_vec"].as<std::vector<double>>();
                cam_dist_coeffs_eigen << cam_dist_coeffs_vec[0], cam_dist_coeffs_vec[1], cam_dist_coeffs_vec[2], cam_dist_coeffs_vec[3], cam_dist_coeffs_vec[4];
            } catch (...) {
                LOG(ERROR) << "[ImageFormatProcessor]: file " << sensor_config_file << " has no camera distort coefficients, and the distort coefficients are set to zero.";
                param_flg_need_undistort = false;
                cam_dist_coeffs_eigen.setZero();
            }
        } else {
            cam_dist_coeffs_eigen.setZero();
        }

        if (param_flg_need_resize) {
            try {
                resize_rows = config_node["cam_resize_rows"].as<int>();
                resize_cols = config_node["cam_resize_cols"].as<int>();
            } catch (...) {
                LOG(ERROR) << "[ImageFormatProcessor]: file " << sensor_config_file << " has no camera new size.";
                param_flg_need_resize = false;
            }
        }

        set_parameter_manually(param_flg_need_resize, param_flg_need_undistort, param_flg_need_equalize, param_flg_rgb_or_bgr_or_gray, cam_intrinsic_eigen, cam_dist_coeffs_eigen, resize_cols,
                               resize_rows);
    }

    // 输出去除畸变后的灰度图像
    cv::Mat process(cv::Mat &raw_img)
    {
        // cv::imshow("raw_img", raw_img);
        if (flg_first_img_) {
            set_parameter_accroding_to_first_img(raw_img);
            flg_first_img_ = false;
        }
        // 进行缩放与去畸变
        cv::Mat img_aft_resize_undist;
        if (param_flg_need_undistort_) { // undistort+resize
            cv::remap(raw_img, img_aft_resize_undist, undist_map1_, undist_map2_, cv::INTER_LINEAR);
        } else if (param_flg_need_resize_) { // only resize
            cv::resize(raw_img, img_aft_resize_undist, resize_size_);
        } else { // no change
            img_aft_resize_undist = raw_img;
        }
        // cv::imshow("img_aft_resize_undist", img_aft_resize_undist);

        // 彩色变灰度
        cv::Mat img_aft_cvt;
        if (param_flg_rgb_or_bgr_or_gray_ == 1) { // rbg->gray
            cv::cvtColor(img_aft_resize_undist, img_aft_cvt, cv::COLOR_RGB2GRAY);
        } else if (param_flg_rgb_or_bgr_or_gray_ == 2) { // bgr->gray
            cv::cvtColor(img_aft_resize_undist, img_aft_cvt, cv::COLOR_BGR2GRAY);
        } else { // gray->gray
            img_aft_cvt = img_aft_resize_undist;
        }
        // cv::imshow("img_aft_cvt", img_aft_cvt);

        // 假设只使用灰度图
        cv::Mat img_aft_equalize;
        if (param_flg_need_equalize_) {
            img_aft_equalize = image_equalize(img_aft_cvt, 3.0);
        } else {
            if (!param_flg_need_undistort_ && !param_flg_need_resize_ && (param_flg_rgb_or_bgr_or_gray_ == 0)) {
                img_aft_equalize = img_aft_cvt.clone();
            } else {
                img_aft_equalize = img_aft_cvt;
            }
        }
        // cv::imshow("img_aft_equalize", img_aft_equalize);
        // cv::waitKey(0);
        return img_aft_equalize;
    }

    void set_parameter_manually(bool param_flg_need_resize, bool param_flg_need_undistort, bool param_flg_need_equalize, int param_flg_rgb_or_bgr_or_gray, Eigen::Matrix3d cam_intrinsic_eigen,
                                Eigen::Matrix<double, 5, 1> cam_dist_coeffs_eigen, int resize_cols, int resize_rows)
    {
        param_flg_need_resize_ = param_flg_need_resize;
        param_flg_need_undistort_ = param_flg_need_undistort;
        param_flg_need_equalize_ = param_flg_need_equalize;
        param_flg_rgb_or_bgr_or_gray_ = param_flg_rgb_or_bgr_or_gray;
        cam_intrinsic_eigen_ = cam_intrinsic_eigen;
        cam_dist_coeffs_eigen_ = cam_dist_coeffs_eigen;
        cv::eigen2cv(cam_intrinsic_eigen_, cam_intrinsic_cv_);
        cv::eigen2cv(cam_dist_coeffs_eigen_, cam_dist_coeffs_cv_);
        resize_size_ = cv::Size(resize_cols, resize_rows);
    }

    int get_undist_img_cols()
    {
        return resize_size_.width;
    }
    int get_undist_img_rows()
    {
        return resize_size_.height;
    }
    Eigen::Matrix3d get_undist_img_intrinsic()
    {
        return cam_undist_intrinsic_eigen_;
    }

private:
    void set_parameter_accroding_to_first_img(cv::Mat &raw_img)
    {
        // 进行缩放与去畸变
        if (!param_flg_need_resize_) {
            resize_size_ = cv::Size(raw_img.cols, raw_img.rows);
        }
        if (param_flg_need_undistort_) {
            cv::Size imageSize(raw_img.cols, raw_img.rows);
            cam_undist_intrinsic_cv_ = getOptimalNewCameraMatrix(cam_intrinsic_cv_, cam_dist_coeffs_cv_, cv::Size(raw_img.cols, raw_img.rows), 0.0 /* 1.0 */, resize_size_, 0);
            cv::initUndistortRectifyMap(cam_intrinsic_cv_, cam_dist_coeffs_cv_, cv::Mat(), cam_undist_intrinsic_cv_, resize_size_, CV_16SC2, undist_map1_, undist_map2_);
            cv::cv2eigen(cam_undist_intrinsic_cv_, cam_undist_intrinsic_eigen_);
        } else {
            cam_undist_intrinsic_cv_ = cam_intrinsic_cv_;
            cv::cv2eigen(cam_undist_intrinsic_cv_, cam_undist_intrinsic_eigen_);
        }
    }

    // 图像直方图均衡（子函数）
    cv::Mat image_equalize(cv::Mat &img, int amp)
    {
        cv::Mat img_temp;
        cv::Size eqa_img_size = cv::Size(std::max(img.cols * 32.0 / 640, 4.0), std::max(img.cols * 32.0 / 640, 4.0));
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(amp, eqa_img_size);
        // Equalize gray image.
        clahe->apply(img, img_temp);
        return img_temp;
    }

    // 图像直方图均衡（子函数）
    cv::Mat equalize_color_image_Ycrcb(cv::Mat &image)
    {
        cv::Mat hist_equalized_image;
        if (param_flg_rgb_or_bgr_or_gray_ == 1) {
            cv::cvtColor(image, hist_equalized_image, cv::COLOR_RGB2YCrCb);
        } else { // param_flg_rgb_or_bgr_or_gray_ == 2
            cv::cvtColor(image, hist_equalized_image, cv::COLOR_BGR2YCrCb);
        }

        // Split the image into 3 channels; Y, Cr and Cb channels respectively and store it in a std::vector
        std::vector<cv::Mat> vec_channels;
        cv::split(hist_equalized_image, vec_channels);

        // Equalize the histogram of only the Y channel
        // cv::equalizeHist(vec_channels[0], vec_channels[0]);
        vec_channels[0] = image_equalize(vec_channels[0], 1);
        cv::merge(vec_channels, hist_equalized_image);
        cv::cvtColor(hist_equalized_image, hist_equalized_image, cv::COLOR_YCrCb2BGR);
        return hist_equalized_image;
    }

    bool param_flg_need_resize_;
    bool param_flg_need_undistort_;
    bool param_flg_need_equalize_;
    int param_flg_rgb_or_bgr_or_gray_; // 0: gray, 1: rgb 2: bgr

    bool flg_first_img_;

    Eigen::Matrix3d cam_intrinsic_eigen_;
    Eigen::Matrix3d cam_undist_intrinsic_eigen_;
    Eigen::Matrix<double, 5, 1> cam_dist_coeffs_eigen_;

    cv::Mat cam_intrinsic_cv_;
    cv::Mat cam_undist_intrinsic_cv_;
    cv::Mat cam_dist_coeffs_cv_;

    cv::Mat undist_map1_, undist_map2_;
    cv::Size resize_size_;
};