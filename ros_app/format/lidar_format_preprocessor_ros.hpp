#pragma once
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

#include <iostream>
#include <algorithm> 
#include <glog/logging.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver/CustomMsg.h>

#include "../../src/format/lidar_format_processor.hpp"

// 功能描述: 
	// 将 rosbag 中读出来的点云msg转换为 pcl::PCLPointCloud2::Ptr 类型。其中绝大
	// 部分数据集的点云不需要做其他处理即可直接 给到 Lidar_Format_Processor，但是部
	// 分数据需要进行另外的处理，然后转换为 pcl::PCLPointCloud2::Ptr 类型，最终给到
	// Lidar_Format_Processor，其中需要另外处理的包括：
		// LIDAR_TYPE_labs_livox
		// LIDAR_TYPE_labs_velodyne_VLP16
		// LIDAR_TYPE_newercollege_ouster_OS1
		// LIDAR_TYPE_ntuviral_ouster_OS1_handler
		// LIDAR_TYPE_urbannav_new_velodyne_HDL32E
		// LIDAR_TYPE_hilti_hesai_PandarXT32
	// 另外，Lidar_Format_Preprocessor_Ros 依赖 Lidar_Format_Processor

class Lidar_Format_Preprocessor_Ros : public Lidar_Format_Processor {
public:
    Lidar_Format_Preprocessor_Ros(){};
	
	Data process(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string = std::string());
	Data process(const livox_ros_driver::CustomMsg::ConstPtr &msg, const std::string& label_string = std::string());
private:
	Data baidu_hesai_Pandar40P_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string = std::string());
	Data baidu_velodyne_HDL32E_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string = std::string());
	Data labs_robosense_RSLiDAR16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string = std::string());
	Data labs_velodyne_VLP16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string = std::string());
	Data urbannav_velodyne_HDL32E_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string = std::string());
	Data urbannav_new_velodyne_HDL32E_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string = std::string());
	Data nclt_velodyne_HDL32E_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string = std::string());
	Data urbanloco_velodyne_HDL32E_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string = std::string());
	Data hilti_hesai_PandarXT32_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string = std::string());
	Data newercollege_ouster_OS1_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string = std::string());
	Data ntuviral_ouster_OS1_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string = std::string());
	Data labs_livox_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg, const std::string& label_string = std::string());
};

Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, const std::string& label_string){
	return labs_livox_handler(msg, label_string); // LIDAR_TYPE_labs_livox
}

Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::process(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string){
	switch (param_lidar_type_){
		case LIDAR_TYPE_baidu_hesai_Pandar40P:
			return baidu_hesai_Pandar40P_handler(msg, label_string);
			break;
		case LIDAR_TYPE_baidu_velodyne_HDL32E:
			return baidu_velodyne_HDL32E_handler(msg, label_string);
			break;
		case LIDAR_TYPE_labs_robosense_RSLiDAR16:
			return labs_robosense_RSLiDAR16_handler(msg, label_string);
			break;
		case LIDAR_TYPE_labs_velodyne_VLP16:
			return labs_velodyne_VLP16_handler(msg, label_string);
			break;
		case LIDAR_TYPE_urbannav_velodyne_HDL32E:
			return urbannav_velodyne_HDL32E_handler(msg, label_string);
			break;
		case LIDAR_TYPE_urbannav_new_velodyne_HDL32E:
			return urbannav_new_velodyne_HDL32E_handler(msg, label_string);
			break;
		case LIDAR_TYPE_urbanloco_velodyne_HDL32E:
			return urbanloco_velodyne_HDL32E_handler(msg, label_string);
			break;
		case LIDAR_TYPE_hilti_hesai_PandarXT32:
			return hilti_hesai_PandarXT32_handler(msg, label_string);
			break;
		case LIDAR_TYPE_newercollege_ouster_OS1:
			return newercollege_ouster_OS1_handler(msg, label_string);
			break;
		case LIDAR_TYPE_ntuviral_ouster_OS1:
			return ntuviral_ouster_OS1_handler(msg, label_string);
			break;
		case LIDAR_TYPE_nclt_new_velodyne_HDL32E:
			return nclt_velodyne_HDL32E_handler(msg, label_string);
			break;
		default:
			LOG(FATAL) << "未定义的点云类型" << std::endl;
	}
}

Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::baidu_hesai_Pandar40P_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string) {
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl_conversions::toPCL(*msg, *cloud2);
	return Lidar_Format_Processor::baidu_hesai_Pandar40P_handler(msg->header.stamp.toSec(), cloud2, label_string);
}

Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::baidu_velodyne_HDL32E_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string) {
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl_conversions::toPCL(*msg, *cloud2);
	return Lidar_Format_Processor::baidu_velodyne_HDL32E_handler(msg->header.stamp.toSec(), cloud2, label_string);
}

// 需另外处理
Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::labs_livox_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg, const std::string& label_string) {
	pcl::PointCloud<PointXYZIRCT> cloud;
	cloud.resize(msg->point_num);
	double time_begin = msg->header.stamp.toSec();
	for(int i = 0; i < msg->point_num; i++) {
		cloud[i].x = msg->points[i].x;
		cloud[i].y = msg->points[i].y;
		cloud[i].z = msg->points[i].z;
		cloud[i].intensity = msg->points[i].reflectivity;
		cloud[i].row = msg->points[i].line;
		cloud[i].timestamp = time_begin + msg->points[i].offset_time / 1000000000.0;
	}
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2<PointXYZIRCT>(cloud, *cloud2);
	return Lidar_Format_Processor::labs_livox_handler(msg->header.stamp.toSec(), cloud2, label_string);
}

// 需另外处理
Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::hilti_hesai_PandarXT32_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string) {
	pcl::PointCloud<Hesai::PointXYZITR> cloud;
	pcl::fromROSMsg(*msg, cloud);
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2<Hesai::PointXYZITR>(cloud, *cloud2);
	return Lidar_Format_Processor::hilti_hesai_PandarXT32_handler(msg->header.stamp.toSec(), cloud2, label_string);
}

// 需另外处理
Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::urbannav_new_velodyne_HDL32E_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string) {
	pcl::PointCloud<Velodyne::PointXYZIRT> cloud;
	pcl::fromROSMsg(*msg, cloud);
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2<Velodyne::PointXYZIRT>(cloud, *cloud2);
	return Lidar_Format_Processor::urbannav_new_velodyne_HDL32E_handler(msg->header.stamp.toSec(), cloud2, label_string);
}

Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::nclt_velodyne_HDL32E_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string) {
	pcl::PointCloud<Velodyne::PointXYZRT> cloud;
	pcl::fromROSMsg(*msg, cloud);
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2<Velodyne::PointXYZRT>(cloud, *cloud2);
	return Lidar_Format_Processor::nclt_velodyne_HDL32E_handler(msg->header.stamp.toSec(), cloud2, label_string);
}

// 需另外处理
Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::newercollege_ouster_OS1_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string) {
	pcl::PointCloud<Ouster::PointXYZITRRNR> cloud;
	pcl::fromROSMsg(*msg, cloud);
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2<Ouster::PointXYZITRRNR>(cloud, *cloud2);
	return Lidar_Format_Processor::newercollege_ouster_OS1_handler(msg->header.stamp.toSec(), cloud2, label_string);
}

// 需另外处理
Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::ntuviral_ouster_OS1_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string) {
	pcl::PointCloud<Ouster::PointXYZITRRNR> cloud;
	pcl::fromROSMsg(*msg, cloud);
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2<Ouster::PointXYZITRRNR>(cloud, *cloud2);
	return Lidar_Format_Processor::ntuviral_ouster_OS1_handler(msg->header.stamp.toSec(), cloud2, label_string);
}

Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::labs_robosense_RSLiDAR16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string) {
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl_conversions::toPCL(*msg, *cloud2);
	return Lidar_Format_Processor::labs_robosense_RSLiDAR16_handler(msg->header.stamp.toSec(), cloud2, label_string);
}

// 需另外处理
Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::labs_velodyne_VLP16_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string) {
	pcl::PointCloud<Velodyne::PointXYZIRT> cloud;
	pcl::fromROSMsg(*msg, cloud);
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2<Velodyne::PointXYZIRT>(cloud, *cloud2);
	return Lidar_Format_Processor::labs_velodyne_VLP16_handler(msg->header.stamp.toSec(), cloud2, label_string);
}

Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::urbannav_velodyne_HDL32E_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string) {
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl_conversions::toPCL(*msg, *cloud2);
	return Lidar_Format_Processor::urbannav_velodyne_HDL32E_handler(msg->header.stamp.toSec(), cloud2, label_string);
}

Lidar_Format_Processor::Data Lidar_Format_Preprocessor_Ros::urbanloco_velodyne_HDL32E_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& label_string) {
	pcl::PointCloud<Velodyne::PointXYZIR> cloud;
	pcl::fromROSMsg(*msg, cloud);
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2<Velodyne::PointXYZIR>(cloud, *cloud2);
	return Lidar_Format_Processor::urbanloco_velodyne_HDL32E_handler(msg->header.stamp.toSec(), cloud2, label_string);
}
