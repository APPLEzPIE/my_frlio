#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#define PUB_NUM 500

struct EIGEN_ALIGN16 OrientPlane {
    PCL_ADD_POINT4D;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;
    double scale_x;
    double scale_y;
    double scale_z;
    uint8_t r;
    uint8_t g;
    uint8_t b;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(OrientPlane,
                                  (float, x, x)(float, y, y)(float, z, z)(double, orientation_x, orientation_x)(double, orientation_y, orientation_y)(double, orientation_z, orientation_z)(
                                      double, orientation_w, orientation_w)(double, scale_x, scale_x)(double, scale_y, scale_y)(double, scale_z, scale_z)(uint8_t, r, r)(uint8_t, g, g)(uint8_t, b, b))

struct EIGEN_ALIGN16 PclPlane {
    PCL_ADD_POINT4D;
    float normal_x;
    float normal_y;
    float normal_z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    float height;
    float width;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(PclPlane, (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z, normal_z)(uint8_t, r, r)(uint8_t, g, g)(
                                                uint8_t, b, b)(float, height, height)(float, width, width))

struct EIGEN_ALIGN16 PclLine {
    PCL_ADD_POINT4D;
    float normal_x;
    float normal_y;
    float normal_z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    float length;
    float radius;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(PclLine, (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z, normal_z)(uint8_t, r, r)(uint8_t, g, g)(
                                               uint8_t, b, b)(float, length, length)(float, radius, radius))

class RvizToolsInterface {
public:
    RvizToolsInterface(ros::NodeHandle &nh, const std::string &frame_id_name)
        : nh_(nh)
        , frame_id_name_(frame_id_name)
        , general_publisher(PUB_NUM)
    {
    }

    void refrash()
    {
        publisher_counter_ = 0;
    }

    void PublishImageGE(const std::string &topic_name, cv::Mat image)
    {
        general_publisher[publisher_counter_] = nh_.advertise<sensor_msgs::Image>(topic_name, 100);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        general_publisher[publisher_counter_].publish(*msg);
        publisher_counter_++;
    }

    template <typename T> void PublishCloudGE(const std::string &topic_name, T cloud_pcl)
    {
        general_publisher[publisher_counter_] = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 100);
        sensor_msgs::PointCloud2 cloud_rosmsg;
        pcl::toROSMsg(*cloud_pcl, cloud_rosmsg);
        cloud_rosmsg.header.frame_id = frame_id_name_;
        general_publisher[publisher_counter_].publish(cloud_rosmsg);
        publisher_counter_++;
    }

    template <typename T> void PublishCloudGE(const std::string &topic_name, T cloud_pcl, double timestamp)
    {
        general_publisher[publisher_counter_] = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 100);
        sensor_msgs::PointCloud2 cloud_rosmsg;
        pcl::toROSMsg(*cloud_pcl, cloud_rosmsg);
        cloud_rosmsg.header.frame_id = frame_id_name_;
        cloud_rosmsg.header.stamp = ros::Time().fromSec(timestamp);
        general_publisher[publisher_counter_].publish(cloud_rosmsg);
        publisher_counter_++;
    }

    void PublishPath(const std::string &topic_name, nav_msgs::Path &path)
    {
        general_publisher[publisher_counter_] = nh_.advertise<nav_msgs::Path>(topic_name, 100);
        general_publisher[publisher_counter_].publish(path);
        publisher_counter_++;
    }

    void PublishOdom(const std::string &topic_name, double timestamp, Eigen::Quaterniond quaternion, Eigen::Vector3d position)
    {
        general_publisher[publisher_counter_] = nh_.advertise<nav_msgs::Odometry>(topic_name, 100);

        nav_msgs::Odometry odom;
        odom.header.frame_id = "/camera_init";
        odom.child_frame_id = "/mapping";
        odom.header.stamp = ros::Time().fromSec(timestamp);
        odom.pose.pose.position.x = position.x();
        odom.pose.pose.position.y = position.y();
        odom.pose.pose.position.z = position.z();
        odom.pose.pose.orientation.x = quaternion.x();
        odom.pose.pose.orientation.y = quaternion.y();
        odom.pose.pose.orientation.z = quaternion.z();
        odom.pose.pose.orientation.w = quaternion.w();

        general_publisher[publisher_counter_].publish(odom);
        publisher_counter_++;
    }

    void PublishLineUseVectorGE(const std::string &topic_name, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud)
    {
        general_publisher[publisher_counter_] = nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 100);
        visualization_msgs::MarkerArray marker_array;
        GetLineMarkerArrayGE(marker_array, cloud, publisher_counter_);
        general_publisher[publisher_counter_].publish(marker_array);
        publisher_counter_++;
    }

    void PublishLineUsePointGE(const std::string &topic_name, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, const Eigen::Vector4d &rgba)
    {
        general_publisher[publisher_counter_] = nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 100);
        visualization_msgs::MarkerArray marker_array;
        GetLineMarkerArrayUsePointGE(marker_array, cloud, rgba);
        general_publisher[publisher_counter_].publish(marker_array);
        publisher_counter_++;
    }

    void PublishPillarUseVectorGE(const std::string &topic_name, pcl::PointCloud<PclLine>::Ptr cloud)
    {
        general_publisher[publisher_counter_] = nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 100);
        visualization_msgs::MarkerArray marker_array;
        GetPillarMarkerArrayGE(marker_array, cloud, publisher_counter_);
        general_publisher[publisher_counter_].publish(marker_array);
        publisher_counter_++;
    }

    void PublishPlaneGE(const std::string &topic_name, pcl::PointCloud<PclPlane>::Ptr cloud)
    {
        general_publisher[publisher_counter_] = nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 100);
        visualization_msgs::MarkerArray marker_array;
        GetPlaneMarkerArrayGE(marker_array, cloud, publisher_counter_);
        general_publisher[publisher_counter_].publish(marker_array);
        publisher_counter_++;
    }

    void PublishPlaneGE(const std::string &topic_name, pcl::PointCloud<OrientPlane>::Ptr cloud)
    {
        general_publisher[publisher_counter_] = nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 100);
        visualization_msgs::MarkerArray marker_array;
        GetPlaneMarkerArrayGE(marker_array, cloud, publisher_counter_);
        general_publisher[publisher_counter_].publish(marker_array);
        publisher_counter_++;
    }

    void PublishCubeGE(const std::string &topic_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float resolution)
    {
        general_publisher[publisher_counter_] = nh_.advertise<visualization_msgs::MarkerArray>(topic_name, 100);
        visualization_msgs::MarkerArray marker_array;
        GetCubeMarkerArrayGE(marker_array, cloud, resolution);
        general_publisher[publisher_counter_].publish(marker_array);
        publisher_counter_++;
    }

    // xyz 代表中心， normal_xyz 代表方向， curvature 代表长度，intensity 代表置信度
    void GetLineMarkerArrayGE(visualization_msgs::MarkerArray &marker_array_out, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_pcl_in, int idx)
    {
        static std::vector<int> max_marker_size(PUB_NUM, 0);
        if (points_pcl_in->empty())
            return;

        marker_array_out.markers.clear();
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = frame_id_name_;
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.ns = "";

        bbox_marker.lifetime = ros::Duration();
        bbox_marker.frame_locked = true;
        bbox_marker.type = visualization_msgs::Marker::CYLINDER;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        int marker_id = 0;

        for (size_t i = 0; i < points_pcl_in->size(); i++) {
            bbox_marker.id = marker_id;
            bbox_marker.pose.position.x = points_pcl_in->points[i].x;
            bbox_marker.pose.position.y = points_pcl_in->points[i].y;
            bbox_marker.pose.position.z = points_pcl_in->points[i].z;
            bbox_marker.color.r = points_pcl_in->points[i].intensity / 1.0;
            bbox_marker.color.g = 0;
            bbox_marker.color.b = 255.0 - points_pcl_in->points[i].curvature / 1.0;
            bbox_marker.color.a = 1.0;
            Eigen::Vector3d vec1(points_pcl_in->points[i].normal_x, points_pcl_in->points[i].normal_y, points_pcl_in->points[i].normal_z);
            vec1.normalize();
            Eigen::Quaterniond orien = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), vec1);
            bbox_marker.pose.orientation.x = orien.x();
            bbox_marker.pose.orientation.y = orien.y();
            bbox_marker.pose.orientation.z = orien.z();
            bbox_marker.pose.orientation.w = orien.w();
            bbox_marker.scale.x = 0.1;
            bbox_marker.scale.y = 0.1;
            bbox_marker.scale.z = points_pcl_in->points[i].curvature;
            marker_array_out.markers.push_back(bbox_marker);
            marker_id++;
        }

        if (marker_array_out.markers.size() > max_marker_size[idx]) {
            max_marker_size[idx] = marker_array_out.markers.size();
        }

        for (size_t i = marker_id; i < max_marker_size[idx]; i++) {
            bbox_marker.id = i;
            bbox_marker.color.a = 0;
            bbox_marker.scale.x = 0;
            bbox_marker.scale.y = 0;
            bbox_marker.scale.z = 0;
            bbox_marker.points.clear();
            marker_array_out.markers.push_back(bbox_marker);
            marker_id++;
        }
    }

    // 相邻两点组成一条直线
    void GetLineMarkerArrayUsePointGE(visualization_msgs::MarkerArray &marker_array_out, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_pcl_in, Eigen::Vector4d color_rgba)
    {
        if (points_pcl_in->empty())
            return;

        marker_array_out.markers.clear();
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = frame_id_name_;
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.ns = "";
        bbox_marker.color.r = color_rgba[0];
        bbox_marker.color.g = color_rgba[1];
        bbox_marker.color.b = color_rgba[2];
        bbox_marker.color.a = 1.0;
        bbox_marker.lifetime = ros::Duration();
        bbox_marker.frame_locked = true;
        bbox_marker.type = visualization_msgs::Marker::LINE_LIST;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        int marker_id = 0;
        bbox_marker.id = marker_id;
        bbox_marker.scale.x = color_rgba[3];
        bbox_marker.scale.y = color_rgba[3];
        bbox_marker.scale.z = 0.1;
        bbox_marker.points.clear();
        geometry_msgs::Point p;
        geometry_msgs::Point p1;
        for (size_t i = 0; i < points_pcl_in->size();) {
            p.x = points_pcl_in->points[i].x;
            p.y = points_pcl_in->points[i].y;
            p.z = points_pcl_in->points[i].z;
            bbox_marker.points.push_back(p);
            i = i + 1;
            p1.x = points_pcl_in->points[i].x;
            p1.y = points_pcl_in->points[i].y;
            p1.z = points_pcl_in->points[i].z;
            bbox_marker.points.push_back(p1);
            i = i + 1;
        }
        marker_array_out.markers.push_back(bbox_marker);
    }

    void GetCubeMarkerArrayGE(visualization_msgs::MarkerArray &marker_array_out, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_pcl_in, float resolution)
    {
        if (points_pcl_in->empty())
            return;

        marker_array_out.markers.clear();
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = frame_id_name_;
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.ns = "";
        bbox_marker.color.a = 0.5;
        bbox_marker.lifetime = ros::Duration();
        bbox_marker.frame_locked = true;
        bbox_marker.type = visualization_msgs::Marker::CUBE_LIST;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        int marker_id = 0;
        bbox_marker.id = marker_id;
        bbox_marker.scale.x = resolution;
        bbox_marker.scale.y = resolution;
        bbox_marker.scale.z = resolution;
        bbox_marker.points.clear();
        geometry_msgs::Point p;
        std_msgs::ColorRGBA color;
        for (size_t i = 0; i < points_pcl_in->size(); i++) {
            p.x = points_pcl_in->points[i].x;
            p.y = points_pcl_in->points[i].y;
            p.z = points_pcl_in->points[i].z;
            color.r = points_pcl_in->points[i].r;
            color.g = points_pcl_in->points[i].g;
            color.b = points_pcl_in->points[i].b;
            color.a = 0.5;
            bbox_marker.points.push_back(p);
            bbox_marker.colors.push_back(color);
        }
        marker_array_out.markers.push_back(bbox_marker);
    }

    // xyz 代表中心， normal_xyz 代表方向， curvature 代表长度，intensity 代表置信度
    void GetPlaneMarkerArrayGE(visualization_msgs::MarkerArray &marker_array_out, const pcl::PointCloud<pcl::PointXYZINormal>::Ptr points_pcl_in, int idx)
    {
        static std::vector<int> max_marker_size(PUB_NUM, 0);
        if (points_pcl_in->empty())
            return;

        marker_array_out.markers.clear();
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = frame_id_name_;
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.ns = "";

        bbox_marker.lifetime = ros::Duration();
        bbox_marker.frame_locked = true;
        bbox_marker.type = visualization_msgs::Marker::CUBE;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        int marker_id = 0;
        for (size_t i = 0; i < points_pcl_in->size(); i++) {
            bbox_marker.id = marker_id;
            bbox_marker.pose.position.x = points_pcl_in->points[i].x;
            bbox_marker.pose.position.y = points_pcl_in->points[i].y;
            bbox_marker.pose.position.z = points_pcl_in->points[i].z;
            Eigen::Vector3d vec1(points_pcl_in->points[i].normal_x, points_pcl_in->points[i].normal_y, points_pcl_in->points[i].normal_z);
            vec1.normalize();
            Eigen::Quaterniond orien = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), vec1);
            bbox_marker.pose.orientation.x = orien.x();
            bbox_marker.pose.orientation.y = orien.y();
            bbox_marker.pose.orientation.z = orien.z();
            bbox_marker.pose.orientation.w = orien.w();
            // bbox_marker.color.r = points_pcl_in->points[i].intensity*255.0;
            // bbox_marker.color.g = 255.0 - points_pcl_in->points[i].intensity*255.0;
            bbox_marker.color.r = 0;
            bbox_marker.color.g = 255.0;
            bbox_marker.color.b = 0;
            bbox_marker.color.a = 0.5;
            bbox_marker.scale.x = points_pcl_in->points[i].curvature;
            bbox_marker.scale.y = points_pcl_in->points[i].intensity;
            bbox_marker.scale.z = 0.005;
            marker_array_out.markers.push_back(bbox_marker);
            marker_id++;
        }
        if (marker_array_out.markers.size() > max_marker_size[idx]) {
            max_marker_size[idx] = marker_array_out.markers.size();
        }

        for (size_t i = marker_id; i < max_marker_size[idx]; i++) {
            bbox_marker.id = i;
            bbox_marker.color.a = 0;
            bbox_marker.scale.x = 0;
            bbox_marker.scale.y = 0;
            bbox_marker.scale.z = 0;
            bbox_marker.points.clear();
            marker_array_out.markers.push_back(bbox_marker);
            marker_id++;
        }
    }

    void GetPlaneMarkerArrayGE(visualization_msgs::MarkerArray &marker_array_out, const pcl::PointCloud<OrientPlane>::Ptr points_pcl_in, int idx)
    {
        static std::vector<int> max_marker_size(PUB_NUM, 0);
        if (points_pcl_in->empty())
            return;

        marker_array_out.markers.clear();
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = frame_id_name_;
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.ns = "";

        bbox_marker.lifetime = ros::Duration();
        bbox_marker.frame_locked = true;
        bbox_marker.type = visualization_msgs::Marker::CUBE;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        int marker_id = 0;
        for (size_t i = 0; i < points_pcl_in->size(); i++) {
            bbox_marker.id = marker_id;
            bbox_marker.pose.position.x = points_pcl_in->points[i].x;
            bbox_marker.pose.position.y = points_pcl_in->points[i].y;
            bbox_marker.pose.position.z = points_pcl_in->points[i].z;
            bbox_marker.pose.orientation.x = points_pcl_in->points[i].orientation_x;
            bbox_marker.pose.orientation.y = points_pcl_in->points[i].orientation_y;
            bbox_marker.pose.orientation.z = points_pcl_in->points[i].orientation_z;
            bbox_marker.pose.orientation.w = points_pcl_in->points[i].orientation_w;
            bbox_marker.scale.x = points_pcl_in->points[i].scale_x;
            bbox_marker.scale.y = points_pcl_in->points[i].scale_y;
            bbox_marker.scale.z = points_pcl_in->points[i].scale_z;
            bbox_marker.color.r = points_pcl_in->points[i].r / 255.0;
            bbox_marker.color.g = points_pcl_in->points[i].g / 255.0;
            bbox_marker.color.b = points_pcl_in->points[i].b / 255.0;
            bbox_marker.color.a = 0.7;
            marker_array_out.markers.push_back(bbox_marker);
            marker_id++;
        }
        if (marker_array_out.markers.size() > max_marker_size[idx]) {
            max_marker_size[idx] = marker_array_out.markers.size();
        }

        for (size_t i = marker_id; i < max_marker_size[idx]; i++) {
            bbox_marker.id = i;
            bbox_marker.color.a = 0;
            bbox_marker.scale.x = 0;
            bbox_marker.scale.y = 0;
            bbox_marker.scale.z = 0;
            bbox_marker.points.clear();
            marker_array_out.markers.push_back(bbox_marker);
            marker_id++;
        }
    }

    void GetPlaneMarkerArrayGE(visualization_msgs::MarkerArray &marker_array_out, const pcl::PointCloud<PclPlane>::Ptr points_pcl_in, int idx)
    {
        static std::vector<int> max_marker_size(PUB_NUM, 0);
        if (points_pcl_in->empty())
            return;

        marker_array_out.markers.clear();
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = frame_id_name_;
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.ns = "";

        bbox_marker.lifetime = ros::Duration();
        bbox_marker.frame_locked = true;
        bbox_marker.type = visualization_msgs::Marker::CUBE;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        int marker_id = 0;
        for (size_t i = 0; i < points_pcl_in->size(); i++) {
            bbox_marker.id = marker_id;
            bbox_marker.pose.position.x = points_pcl_in->points[i].x;
            bbox_marker.pose.position.y = points_pcl_in->points[i].y;
            bbox_marker.pose.position.z = points_pcl_in->points[i].z;
            Eigen::Vector3d vec1(points_pcl_in->points[i].normal_x, points_pcl_in->points[i].normal_y, points_pcl_in->points[i].normal_z);
            vec1.normalize();
            Eigen::Quaterniond orien = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), vec1);
            bbox_marker.pose.orientation.x = orien.x();
            bbox_marker.pose.orientation.y = orien.y();
            bbox_marker.pose.orientation.z = orien.z();
            bbox_marker.pose.orientation.w = orien.w();
            bbox_marker.color.r = points_pcl_in->points[i].r;
            bbox_marker.color.g = points_pcl_in->points[i].g;
            bbox_marker.color.b = points_pcl_in->points[i].b;
            bbox_marker.color.a = 0.5;
            bbox_marker.scale.x = points_pcl_in->points[i].width;
            bbox_marker.scale.y = points_pcl_in->points[i].height;
            bbox_marker.scale.z = 0.005;
            marker_array_out.markers.push_back(bbox_marker);
            marker_id++;
        }
        if (marker_array_out.markers.size() > max_marker_size[idx]) {
            max_marker_size[idx] = marker_array_out.markers.size();
        }

        for (size_t i = marker_id; i < max_marker_size[idx]; i++) {
            bbox_marker.id = i;
            bbox_marker.color.a = 0;
            bbox_marker.scale.x = 0;
            bbox_marker.scale.y = 0;
            bbox_marker.scale.z = 0;
            bbox_marker.points.clear();
            marker_array_out.markers.push_back(bbox_marker);
            marker_id++;
        }
    }

    void GetPillarMarkerArrayGE(visualization_msgs::MarkerArray &marker_array_out, const pcl::PointCloud<PclLine>::Ptr points_pcl_in, int idx)
    {
        static std::vector<int> max_marker_size(PUB_NUM, 0);
        if (points_pcl_in->empty())
            return;

        marker_array_out.markers.clear();
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = frame_id_name_;
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.ns = "";

        bbox_marker.lifetime = ros::Duration();
        bbox_marker.frame_locked = true;
        bbox_marker.type = visualization_msgs::Marker::CYLINDER;
        bbox_marker.action = visualization_msgs::Marker::ADD;
        int marker_id = 0;

        for (size_t i = 0; i < points_pcl_in->size(); i++) {
            bbox_marker.id = marker_id;
            bbox_marker.pose.position.x = points_pcl_in->points[i].x;
            bbox_marker.pose.position.y = points_pcl_in->points[i].y;
            bbox_marker.pose.position.z = points_pcl_in->points[i].z;
            bbox_marker.color.r = points_pcl_in->points[i].r;
            bbox_marker.color.g = points_pcl_in->points[i].g;
            bbox_marker.color.b = points_pcl_in->points[i].b;
            bbox_marker.color.a = 0.6;
            Eigen::Vector3d vec1(points_pcl_in->points[i].normal_x, points_pcl_in->points[i].normal_y, points_pcl_in->points[i].normal_z);
            vec1.normalize();
            Eigen::Quaterniond orien = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), vec1);
            bbox_marker.pose.orientation.x = orien.x();
            bbox_marker.pose.orientation.y = orien.y();
            bbox_marker.pose.orientation.z = orien.z();
            bbox_marker.pose.orientation.w = orien.w();
            bbox_marker.scale.x = points_pcl_in->points[i].radius;
            bbox_marker.scale.y = points_pcl_in->points[i].radius;
            bbox_marker.scale.z = points_pcl_in->points[i].length;
            marker_array_out.markers.push_back(bbox_marker);
            marker_id++;
        }

        if (marker_array_out.markers.size() > max_marker_size[idx]) {
            max_marker_size[idx] = marker_array_out.markers.size();
        }

        for (size_t i = marker_id; i < max_marker_size[idx]; i++) {
            bbox_marker.id = i;
            bbox_marker.color.a = 0;
            bbox_marker.scale.x = 0;
            bbox_marker.scale.y = 0;
            bbox_marker.scale.z = 0;
            bbox_marker.points.clear();
            marker_array_out.markers.push_back(bbox_marker);
            marker_id++;
        }
    }

    // void GenColorPoint(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr debug_cloud_out, const Point& point, const int rgb_counter) {
    // 	pcl::PointXYZRGBNormal pointrgb;
    //     pointrgb.x = point.center[0];
    //     pointrgb.y = point.center[1];
    //     pointrgb.z = point.center[2];
    //     if (rgb_counter == 0){
    //         pointrgb.r = 0;
    //         pointrgb.g = 0;
    //         pointrgb.b = 255;
    //     }else if (rgb_counter == 1){
    //         pointrgb.r = 255;
    //         pointrgb.g = 0;
    //         pointrgb.b = 0;
    //     }else if (rgb_counter == 2){
    //         pointrgb.r = 0;
    //         pointrgb.g = 255;
    //         pointrgb.b = 0;
    //     }else if (rgb_counter == 3){
    //         pointrgb.r = 255;
    //         pointrgb.g = 255;
    //         pointrgb.b = 0;
    //     }else if (rgb_counter == 4){
    //         pointrgb.r = 0;
    //         pointrgb.g = 255;
    //         pointrgb.b = 255;
    //     }else if (rgb_counter == 5){
    //         pointrgb.r = 255;
    //         pointrgb.g = 0;
    //         pointrgb.b = 255;
    //     }
    //     debug_cloud_out->push_back(pointrgb);
    // }

    std::vector<ros::Publisher> general_publisher;
    ros::NodeHandle nh_;
    int publisher_counter_;
    std::string frame_id_name_;
};

#