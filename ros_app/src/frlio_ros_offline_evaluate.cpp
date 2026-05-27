#include <iostream>
#include <string>
#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <gflags/gflags.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>

#include "./rviz_tools.hpp"
#include "../../src/tools/tools.hpp"
#include "../../src/tools/time_tools.hpp"

#include "../pause_tools/pause_tools.hpp"

#include "../format/lidar_format_preprocessor_ros.hpp"
#include "../../src/format/imu_format_processor.hpp"
#include "../../src/system/fr_lio.h"

template <typename PointType> class Ros_Msg_Handle_Interface {
public:
    Ros_Msg_Handle_Interface(Odometry<PointType> *odometry_ptr)
        : odometry_ptr_(odometry_ptr)
    {
    }

    Ros_Msg_Handle_Interface(Odometry<PointType> *odometry_ptr, Lidar_Format_Preprocessor_Ros *lidar_format_preprocessor_ros_ptr, Imu_Format_Processor *imu_format_processor_ptr)
        : odometry_ptr_(odometry_ptr)
        , lidar_format_preprocessor_ros_ptr_(lidar_format_preprocessor_ros_ptr)
        , imu_format_processor_ptr_(imu_format_processor_ptr)
    {
    }

    virtual void handle(rosbag::MessageInstance &m) = 0;

    Odometry<PointType> *odometry_ptr_;
    Lidar_Format_Preprocessor_Ros *lidar_format_preprocessor_ros_ptr_;
    Imu_Format_Processor *imu_format_processor_ptr_;
};

template <typename PointType> class Livox_Ros_Msg_Handle : public Ros_Msg_Handle_Interface<PointType> {
public:
    Livox_Ros_Msg_Handle(Odometry<PointType> *odometry_ptr)
        : Ros_Msg_Handle_Interface<PointType>(odometry_ptr)
    {
    }

    Livox_Ros_Msg_Handle(Odometry<PointType> *odometry_ptr, Lidar_Format_Preprocessor_Ros *lidar_format_preprocessor_ros_ptr, Imu_Format_Processor *imu_format_processor_ptr)
        : Ros_Msg_Handle_Interface<PointType>(odometry_ptr, lidar_format_preprocessor_ros_ptr, imu_format_processor_ptr)
    {
    }

    virtual void handle(rosbag::MessageInstance &m)
    {
        livox_ros_driver::CustomMsg::ConstPtr msg = m.instantiate<livox_ros_driver::CustomMsg>();
        Lidar_Format_Processor::Data data = Ros_Msg_Handle_Interface<PointType>::lidar_format_preprocessor_ros_ptr_->process(msg);
        std::shared_ptr<Lidar_Data<PointXYZIRCT>> lidar_data(new Lidar_Data<PointXYZIRCT>(data.begin_time, data.end_time, data.cloud));
        Ros_Msg_Handle_Interface<PointType>::odometry_ptr_->add_lidar_data(lidar_data);
    }
};

template <typename PointType> class Pcl_Ros_Msg_Handle : public Ros_Msg_Handle_Interface<PointType> {
public:
    Pcl_Ros_Msg_Handle(Odometry<PointType> *odometry_ptr)
        : Ros_Msg_Handle_Interface<PointType>(odometry_ptr)
    {
    }

    Pcl_Ros_Msg_Handle(Odometry<PointType> *odometry_ptr, Lidar_Format_Preprocessor_Ros *lidar_format_preprocessor_ros_ptr, Imu_Format_Processor *imu_format_processor_ptr)
        : Ros_Msg_Handle_Interface<PointType>(odometry_ptr, lidar_format_preprocessor_ros_ptr, imu_format_processor_ptr)
    {
    }

    virtual void handle(rosbag::MessageInstance &m)
    {
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        Lidar_Format_Processor::Data data = Ros_Msg_Handle_Interface<PointType>::lidar_format_preprocessor_ros_ptr_->process(msg);
        std::shared_ptr<Lidar_Data<PointXYZIRCT>> lidar_data(new Lidar_Data<PointXYZIRCT>(data.begin_time, data.end_time, data.cloud));
        Ros_Msg_Handle_Interface<PointType>::odometry_ptr_->add_lidar_data(lidar_data);
    }
};

template <typename PointType> class Imu_Ros_Msg_Handle : public Ros_Msg_Handle_Interface<PointType> {
public:
    Imu_Ros_Msg_Handle(Odometry<PointType> *odometry_ptr)
        : Ros_Msg_Handle_Interface<PointType>(odometry_ptr)
    {
    }

    Imu_Ros_Msg_Handle(Odometry<PointType> *odometry_ptr, Lidar_Format_Preprocessor_Ros *lidar_format_preprocessor_ros_ptr, Imu_Format_Processor *imu_format_processor_ptr)
        : Ros_Msg_Handle_Interface<PointType>(odometry_ptr, lidar_format_preprocessor_ros_ptr, imu_format_processor_ptr)
    {
    }

    virtual void handle(rosbag::MessageInstance &m)
    {
        sensor_msgs::Imu::ConstPtr msg = m.instantiate<sensor_msgs::Imu>();
        std::shared_ptr<Imu_Data> imu_data(new Imu_Data(msg->header.stamp.toSec(), msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z, msg->angular_velocity.x,
                                                        msg->angular_velocity.y, msg->angular_velocity.z));
        Ros_Msg_Handle_Interface<PointType>::imu_format_processor_ptr_->process(imu_data->acc_);
        Ros_Msg_Handle_Interface<PointType>::odometry_ptr_->add_imu_data(imu_data);
    }
};

DEFINE_string(bag_file, "/media/zas/My Passport/dataset/KITTI/sequences/01/", "");
DEFINE_string(dataset_base_dir, "/home/apple-pie/SLAM/fr-lio/eee_01/", "");
DEFINE_string(evaluate_dir, "/home/apple-pie/SLAM/fr-lio/eee_01/", "");
DEFINE_string(system_dir, "/home/apple-pie/SLAM/fr-lio/eee_01/", "");
DEFINE_string(sensor_config_file, "/home/apple-pie/SLAM/fr-lio/eee_01/", "");

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "fr-lio");
    ros::NodeHandle nh;
    RvizTools rviz_tools(nh, "camera_init");

    signal(SIGINT, SigHandle);
    boost::thread t = boost::thread(boost::bind(&keyboardLoop));

    LOG(INFO) << "[fr-lio][0] Lidar_Format_Processor 初始化" << std::endl;
    Lidar_Format_Preprocessor_Ros lidar_format_preprocessor_ros;
    lidar_format_preprocessor_ros.load_config_parameter(FLAGS_sensor_config_file);

    LOG(INFO) << "[fr-lio][1] Imu_Format_Processor 初始化" << std::endl;
    Imu_Format_Processor imu_format_processor;
    imu_format_processor.load_config_parameter(FLAGS_sensor_config_file);

    LOG(INFO) << "[fr-lio][2] Odometry 初始化" << std::endl;
    Odometry<PointXYZIRCT> odometry(FLAGS_system_dir, FLAGS_sensor_config_file);
    odometry.load_config_parameter();

    LOG(INFO) << "[fr-lio][3] Ros 初始化" << std::endl;
    std::string lidar_topic, imu_topic;
    {
        YAML::Node config_node = YAML::LoadFile(FLAGS_sensor_config_file);
        try {
            lidar_topic = config_node["ros_topic"]["lidar"].as<std::string>();
            imu_topic = config_node["ros_topic"]["imu"].as<std::string>();
        } catch (...) {
            LOG(FATAL) << "[main]: file " << FLAGS_sensor_config_file << " has a bad conversion";
        }
        LOG(INFO) << std::endl
                  << "| --------------- [main parameter] --------------- |" << std::endl
                  << "[lidar_topic] " << lidar_topic << std::endl
                  << "[imu_topic] " << imu_topic << std::endl
                  << std::endl;
    }
    std::unordered_map<std::string, std::shared_ptr<Ros_Msg_Handle_Interface<PointXYZIRCT>>> ros_msg_map;
    if (lidar_format_preprocessor_ros.get_param_lidar_type() == Lidar_Format_Processor::LIDAR_TYPE::LIDAR_TYPE_labs_livox) {
        ros_msg_map.emplace(lidar_topic, std::make_shared<Livox_Ros_Msg_Handle<PointXYZIRCT>>(&odometry, &lidar_format_preprocessor_ros, &imu_format_processor));
    } else {
        ros_msg_map.emplace(lidar_topic, std::make_shared<Pcl_Ros_Msg_Handle<PointXYZIRCT>>(&odometry, &lidar_format_preprocessor_ros, &imu_format_processor));
    }

    ros_msg_map.emplace(imu_topic, std::make_shared<Imu_Ros_Msg_Handle<PointXYZIRCT>>(&odometry, &lidar_format_preprocessor_ros, &imu_format_processor));
    std::vector<std::string> sensor_topics_;
    sensor_topics_.push_back(lidar_topic);
    sensor_topics_.push_back(imu_topic);

    LOG(INFO) << "[fr-lio][4] 读取rosbag文件..." << std::endl;
    rosbag::Bag bag;
    bag.open(FLAGS_bag_file, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(sensor_topics_));

    LOG(INFO) << "[fr-lio][7] 创建轨迹文件夹..." << std::endl;
    std::string str_bag_file = FLAGS_bag_file;
    std::string str_dataset_base_dir = FLAGS_dataset_base_dir;
    if (str_dataset_base_dir.back() != '/') {
        str_dataset_base_dir += "/";
    }
    std::string str_sub_evaluate_file = str_bag_file.substr(str_dataset_base_dir.size());

    int idx_1 = -1;
    while (str_sub_evaluate_file.find("/", idx_1 + 1) < str_sub_evaluate_file.length()) {
        idx_1 = str_sub_evaluate_file.find("/", idx_1 + 1);
    }

    std::string str_evaluate_dir = FLAGS_evaluate_dir + str_sub_evaluate_file.substr(0, idx_1 + 1);
    std::string command = "mkdir -p " + str_evaluate_dir;
    system(command.c_str());

    std::string str_evaluate_file_name = str_sub_evaluate_file.substr(idx_1 + 1);
    int idx_2 = -1;
    while (str_evaluate_file_name.find(".", idx_2 + 1) < str_evaluate_file_name.length()) {
        idx_2 = str_evaluate_file_name.find(".", idx_2 + 1);
    }
    str_evaluate_file_name = str_evaluate_file_name.substr(0, idx_2);

    LOG(INFO) << "str_evaluate_dir: " << str_evaluate_dir << std::endl;
    LOG(INFO) << "str_evaluate_file_name: " << str_evaluate_file_name << std::endl;
    LOG(INFO) << "traj_file: " << str_evaluate_dir + str_evaluate_file_name + "_traj.txt" << std::endl;

    LOG(INFO) << "[fr-lio][6] 系统运行..." << std::endl;

    std::string name;
    int idx = 0;
    std::unordered_map<std::string, std::shared_ptr<Ros_Msg_Handle_Interface<PointXYZIRCT>>>::iterator iter;
    // ros::Rate r(5000);
    for (auto m : view) {
        while (pause_detect()) {
            if (system_shoutdown) {
                break;
            }
        }
        if (system_shoutdown) {
            break;
        }

        name = m.getTopic();
        iter = ros_msg_map.find(name);
        if (iter == ros_msg_map.end()) {
            continue;
        }
        iter->second->handle(m);

        while (1) {
            if (odometry.run()) {
#if MY_DEBUG_MODE
                rviz_tools.PubAll(odometry.map_cloud_tmp_, odometry.predict_undistorted_cloud_, odometry.predict_distorted_cloud_);
#elif MY_VIS_MODE
                rviz_tools.PubAll(odometry.map_cloud_tmp_ /* , odometry.predict_undistorted_cloud_, odometry.predict_distorted_cloud_ */);
#endif
            } else {
                break;
            }
        }
    }

    Timer::print_all_msg();
    odometry.print_all();
    LOG(INFO) << "开始保存轨迹、耗时等等信息" << std::endl;
    Timer::dump_into_file(str_evaluate_dir + str_evaluate_file_name + "_time_usage.txt");
    odometry.dump_into_file(str_evaluate_dir + str_evaluate_file_name + "_traj.txt", str_evaluate_dir + str_evaluate_file_name + "_high_freq_traj.txt",
                            str_evaluate_dir + str_evaluate_file_name + "_state.txt");
    odometry.save_info(str_evaluate_dir + str_evaluate_file_name + "_info.txt");
    // #ifdef MY_DEBUG_MODE
    //     odometry.save_imu_info(FLAGS_my_debug_dir + "debug_imu_info.txt");
    //     odometry.save_dx_info(FLAGS_my_debug_dir + "debug_dx_info.txt");
    // #endif
    LOG(INFO) << "结束保存轨迹、耗时等等信息" << std::endl;
    return 0;
}
