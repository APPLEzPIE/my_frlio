#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <gflags/gflags.h>

#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>

#include <sensor_msgs/image_encodings.h>

#include "../../src/tools/tools.hpp"
#include "../../src/tools/time_tools.hpp"

#include "./rviz_tools.hpp"
#include "../pause_tools/pause_tools.hpp"

#include "../format/lidar_format_preprocessor_ros.hpp"
#include "../../src/format/imu_format_processor.hpp"
#include "../../src/format/image_format_processor.hpp"
#include "../../src/system/fr_lio.h"

/**
 * @brief 标准点云回调函数
 *
 * 该函数作为ROS订阅者的回调函数，用于处理来自激光雷达的点云消息。
 * 它将原始的ROS PointCloud2消息转换为内部的激光雷达数据格式，
 * 然后将处理后的数据添加到里程计模块中进行后续的SLAM计算。
 *
 * @param msg 指向ROS PointCloud2消息的常量指针，包含激光雷达扫描的原始点云数据
 * @param lidar_format_preprocessor_ros 指向激光雷达格式预处理器的指针，
 *                                       负责将ROS消息格式转换为内部数据格式
 * @param odometry 指向里程计对象的指针，用于接收和处理激光雷达数据，
 *                 进行位置和姿态估计
 *
 * @return void
 *
 * @note 该函数在每次接收到新的点云消息时都会被调用
 * @note 处理过程包括：格式转换 -> 数据包装 -> 添加到里程计
 */
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg, Lidar_Format_Preprocessor_Ros *lidar_format_preprocessor_ros, Odometry<PointXYZIRCT> *odometry)
{
    Lidar_Format_Processor::Data data = lidar_format_preprocessor_ros->process(msg);
    std::shared_ptr<Lidar_Data<PointXYZIRCT>> lidar_data(new Lidar_Data<PointXYZIRCT>(data.begin_time, data.end_time, data.cloud));
    odometry->add_lidar_data(lidar_data);
}

/**
 * @brief Livox激光雷达点云回调函数
 *
 * @details 该函数作为Livox激光雷达驱动的ROS消息回调函数，用于处理接收到的
 *          自定义格式的点云消息。函数会进行格式转换，创建带时间戳的激光数据对象，
 *          并将其添加到里程计系统中用于SLAM处理。
 *
 * @param msg 指向Livox自定义消息的常量指针，包含原始激光点云数据
 * @param lidar_format_preprocessor_ros 指向激光格式预处理器的指针，负责将
 *                                        Livox消息转换为标准格式
 * @param odometry 指向里程计对象的指针，用于接收处理后的激光数据并进行SLAM计算
 *
 * @return void
 *
 * @note 该函数通常作为ROS订阅者的回调函数被异步调用
 *
 */
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg, Lidar_Format_Preprocessor_Ros *lidar_format_preprocessor_ros, Odometry<PointXYZIRCT> *odometry)
{
    Lidar_Format_Processor::Data data = lidar_format_preprocessor_ros->process(msg);
    std::shared_ptr<Lidar_Data<PointXYZIRCT>> lidar_data(new Lidar_Data<PointXYZIRCT>(data.begin_time, data.end_time, data.cloud));
    odometry->add_lidar_data(lidar_data);
}

void compressed_img_cbk(const sensor_msgs::CompressedImageConstPtr &msg, ImageFormatProcessor *image_format_processor, Odometry<PointXYZIRCT> *odometry)
{
    cv::Mat raw_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    if (raw_img.channels() == 1) {
        cv::cvtColor(raw_img, raw_img, cv::COLOR_GRAY2BGR);
    }

    cv::Mat img = image_format_processor->process(raw_img);
    if (img.channels() == 1) {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }

    std::shared_ptr<Image_Data> image_data(new Image_Data(msg->header.stamp.toSec(), img));
    odometry->add_image_data(image_data);
}

void img_cbk(const sensor_msgs::Image::ConstPtr &msg, ImageFormatProcessor *image_format_processor, Odometry<PointXYZIRCT> *odometry)
{
    cv::Mat raw_img;
    if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
        raw_img = cv_bridge::toCvShare(msg, "mono8")->image;
        cv::cvtColor(raw_img, raw_img, cv::COLOR_GRAY2BGR);
    } else {
        raw_img = cv_bridge::toCvShare(msg, "bgr8")->image;
    }

    cv::Mat img = image_format_processor->process(raw_img);
    if (img.channels() == 1) {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }

    std::shared_ptr<Image_Data> image_data(new Image_Data(msg->header.stamp.toSec(), img));
    odometry->add_image_data(image_data);
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg, Imu_Format_Processor *imu_format_processor, Odometry<PointXYZIRCT> *odometry)
{
    std::shared_ptr<Imu_Data> imu_data(new Imu_Data(msg->header.stamp.toSec(), msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z, msg->angular_velocity.x,
                                                    msg->angular_velocity.y, msg->angular_velocity.z));
    imu_format_processor->process(imu_data->acc_);
    odometry->add_imu_data(imu_data);
}

DEFINE_string(system_dir, "/media/zas/My Passport/dataset/KITTI/sequences/01/", "");
DEFINE_string(sensor_config_file, "/media/zas/My Passport/dataset/KITTI/sequences/01/", "");
DEFINE_string(my_log_dir, "/media/zas/My Passport/dataset/KITTI/sequences/01/", "");
DEFINE_string(my_debug_dir, "/media/zas/My Passport/dataset/KITTI/sequences/01/", "");

int main(int argc, char *argv[])
{
    //解析命令行
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    //日志配置，设置日志等级和颜色输出，并初始化Google Logging库
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    // ROS节点初始化，创建ROS句柄，并设置Ctrl+C信号处理函数和键盘输入线程
    ros::init(argc, argv, "fr_lio");
    ros::NodeHandle nh;
    RvizTools rviz_tools(nh, "camera_init");
    //设置Ctrl+C信号处理函数，确保程序能够正确响应用户的中断请求
    signal(SIGINT, SigHandle);
    boost::thread t = boost::thread(boost::bind(&keyboardLoop));

    LOG(INFO) << "[fr-lio][0] Lidar_Format_Processor 初始化" << std::endl;
    Lidar_Format_Preprocessor_Ros lidar_format_preprocessor_ros;
    lidar_format_preprocessor_ros.load_config_parameter(FLAGS_sensor_config_file);
    Lidar_Format_Preprocessor_Ros *lidar_format_preprocessor_ros_ptr = &lidar_format_preprocessor_ros;

    LOG(INFO) << "[fr-lio][1] Imu_Format_Processor 初始化" << std::endl;
    Imu_Format_Processor imu_format_processor;
    imu_format_processor.load_config_parameter(FLAGS_sensor_config_file);
    Imu_Format_Processor *imu_format_processor_ptr = &imu_format_processor;

    LOG(INFO) << "[fr-lio][1] ImageFormatProcessor 初始化" << std::endl;
    ImageFormatProcessor image_format_processor;
    image_format_processor.load_config_parameter(FLAGS_sensor_config_file);
    ImageFormatProcessor *image_format_processor_ptr = &image_format_processor;

    LOG(INFO) << "[fr-lio][2] Odometry 初始化" << std::endl;
    Odometry<PointXYZIRCT> odometry(FLAGS_system_dir, FLAGS_sensor_config_file);
    odometry.load_config_parameter();
    Odometry<PointXYZIRCT> *odometry_ptr = &odometry;

    LOG(INFO) << "[fr-lio][3] Ros 初始化" << std::endl;
    std::string lidar_topic, imu_topic, camera_topic;
    bool enable_colorize = false;
    bool save_keyframe_pcd = false;
    bool save_rgb_keyframe_pcd = false;
    std::string save_pcd_dir = FLAGS_my_log_dir + "pcd";
    int save_pcd_stride = 1;
    {
        YAML::Node config_node = YAML::LoadFile(FLAGS_sensor_config_file);
        try {
            lidar_topic = config_node["ros_topic"]["lidar"].as<std::string>();
            imu_topic = config_node["ros_topic"]["imu"].as<std::string>();
            try {
                camera_topic = config_node["ros_topic"]["camera"].as<std::string>();
            } catch (...) {
                camera_topic = config_node["ros_topic"]["img"].as<std::string>();
            }
            enable_colorize = config_node["camera"]["enable_colorize"].as<bool>();

            if (config_node["output"]) {
                try {
                    save_keyframe_pcd = config_node["output"]["save_keyframe_pcd"].as<bool>();
                } catch (...) {
                }
                try {
                    save_rgb_keyframe_pcd = config_node["output"]["save_rgb_keyframe_pcd"].as<bool>();
                } catch (...) {
                }
                try {
                    save_pcd_dir = config_node["output"]["save_pcd_dir"].as<std::string>();
                } catch (...) {
                }
                try {
                    save_pcd_stride = config_node["output"]["save_pcd_stride"].as<int>();
                } catch (...) {
                }
            }
        } catch (...) {
            LOG(FATAL) << "[main]: file " << FLAGS_sensor_config_file << " has a bad conversion";
        }
        if (save_pcd_stride <= 0) {
            save_pcd_stride = 1;
        }

        if (save_keyframe_pcd || save_rgb_keyframe_pcd) {
            boost::filesystem::create_directories(save_pcd_dir);
        }

        LOG(INFO) << std::endl
                  << "| --------------- [main parameter] --------------- |" << std::endl
                  << "[lidar_topic] " << lidar_topic << std::endl
                  << "[imu_topic] " << imu_topic << std::endl
                  << "[camera_topic] " << camera_topic << std::endl
                  << "[camera.enable_colorize] " << enable_colorize << std::endl
                  << "[output.save_keyframe_pcd] " << save_keyframe_pcd << std::endl
                  << "[output.save_rgb_keyframe_pcd] " << save_rgb_keyframe_pcd << std::endl
                  << "[output.save_pcd_dir] " << save_pcd_dir << std::endl
                  << "[output.save_pcd_stride] " << save_pcd_stride << " (ignored in final-map-only mode)" << std::endl
                  << std::endl;
    }
    ros::Subscriber sub_pcl = lidar_format_preprocessor_ros.get_param_lidar_type() == Lidar_Format_Processor::LIDAR_TYPE::LIDAR_TYPE_labs_livox ?
                                  nh.subscribe<livox_ros_driver::CustomMsg>(lidar_topic, 200000, boost::bind(livox_pcl_cbk, _1, lidar_format_preprocessor_ros_ptr, odometry_ptr)) :
                                  nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 200000, boost::bind(standard_pcl_cbk, _1, lidar_format_preprocessor_ros_ptr, odometry_ptr));
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200000, boost::bind(imu_cbk, _1, imu_format_processor_ptr, odometry_ptr));

    ros::Subscriber sub_img;
    if (enable_colorize && !camera_topic.empty()) {
        bool flg_compressed = false;
        std::string camera_topic_tmp = camera_topic;
        while (1) {
            std::size_t idx_curr = camera_topic_tmp.find('/');
            if (idx_curr == std::string::npos) {
                if (camera_topic_tmp == "compressed") {
                    flg_compressed = true;
                }
                break;
            } else {
                camera_topic_tmp = camera_topic_tmp.substr(idx_curr + 1);
            }
        }

        if (flg_compressed) {
            sub_img = nh.subscribe<sensor_msgs::CompressedImage>(camera_topic, 200000, boost::bind(compressed_img_cbk, _1, image_format_processor_ptr, odometry_ptr));
        } else {
            sub_img = nh.subscribe<sensor_msgs::Image>(camera_topic, 200000, boost::bind(img_cbk, _1, image_format_processor_ptr, odometry_ptr));
        }
    }

    LOG(INFO) << "[fr-lio][4] 系统运行..." << std::endl;
    ros::Rate r(5000);
    go = true;
    if (save_keyframe_pcd || save_rgb_keyframe_pcd) {
        LOG(INFO) << "[output] final-map-only save enabled, runtime will not cache submaps." << std::endl;
    }

    while (!system_shoutdown) {
        ros::spinOnce();
        r.sleep();
        if (pause_detect()) {
            continue;
        }

        while (1) {
            if (odometry.run()) {
#if MY_VIS_MODE
                // Feed the current optimized pose to RViz path cache for trajectory rendering.
                const State_Group curr_state = Imu_Integration::get_curr_system_state();
                rviz_tools.AddPath(curr_state.quat_, curr_state.pos_);
                rviz_tools.PubAll(odometry.map_cloud_tmp_ /* , odometry.predict_undistorted_cloud_, odometry.predict_distorted_cloud_ */);
#endif

#ifdef GEN_RGB_MAP
                double timestamp = 0.0;
                Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
                Eigen::Vector3d position = Eigen::Vector3d::Zero();
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_submap;
                while (odometry.get_rgb_submap(timestamp, quaternion, position, rgb_submap)) {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr submap(new pcl::PointCloud<pcl::PointXYZI>());
                    submap->reserve(rgb_submap->size());
                    pcl::PointXYZI pt_tmp;
                    for (auto &pt : rgb_submap->points) {
                        pt_tmp.x = pt.x;
                        pt_tmp.y = pt.y;
                        pt_tmp.z = pt.z;
                        submap->push_back(pt_tmp);
                    }

                    rviz_tools.PublishCloudGE("keyframe_cloud", submap, timestamp);
                    rviz_tools.PublishCloudGE("rgb_keyframe_cloud", rgb_submap, timestamp);
                    rviz_tools.PublishOdom("keyframe_pose", timestamp, quaternion, position);
                }

                cv::Mat overlay_img;
                while (odometry.get_overlay_image(timestamp, overlay_img)) {
                    rviz_tools.PublishImageGE("overlay_image", overlay_img);
                }
#endif
            } else {
                break;
            }
        }
    }

    Timer::print_all_msg();
    std::cout << "start save" << std::endl;

    if (save_keyframe_pcd) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr final_map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        odometry.get_final_map_cloud(final_map_cloud);
        std::string final_map_file = save_pcd_dir + "/final_map.pcd";
        pcl::io::savePCDFileBinaryCompressed(final_map_file, *final_map_cloud);
        LOG(INFO) << "[output] saved final map: " << final_map_file << ", points: " << final_map_cloud->size() << std::endl;
    }

    if (save_rgb_keyframe_pcd) {
        LOG(WARNING) << "[output] final RGB map saving is not supported in final-map-only mode, skip save_rgb_keyframe_pcd." << std::endl;
    }

    Timer::dump_into_file(FLAGS_my_log_dir + "time_usage.txt");
    odometry.dump_into_file(FLAGS_my_log_dir + "low_freq_traj.txt", FLAGS_my_log_dir + "high_freq_traj.txt", FLAGS_my_log_dir + "state.txt");
    odometry.save_info(FLAGS_my_log_dir + "info.txt");
#ifdef MY_DEBUG_MODE
    odometry.save_imu_info(FLAGS_my_debug_dir + "debug_info.txt");
#endif
    std::cout << "end save" << std::endl;
    return 0;
}

Eigen::Vector3d gen_color_for_undegrade_frame(unsigned long long &id)
{
    if (id % 2 == 0) {
        return Eigen::Vector3d(255, 0, 0);
    } else if (id % 2 == 1) {
        return Eigen::Vector3d(255, 255, 0);
    };
}

Eigen::Vector3d gen_color_for_degrade_frame(unsigned long long &id)
{
    if (id % 2 == 0) {
        return Eigen::Vector3d(0, 0, 255);
    } else if (id % 2 == 1) {
        return Eigen::Vector3d(0, 255, 255);
    };
}