#pragma once
#include "../visualization_tools/rviz_tools_Interface.hpp"

class RvizTools : public RvizToolsInterface {
public:
    RvizTools(ros::NodeHandle &nh, const std::string &frame_id_name)
        : RvizToolsInterface(nh, frame_id_name)
    {
    }

    template <typename CloudPtrType1, typename CloudPtrType2, typename CloudPtrType3, typename CloudPtrType4>
    void PubAll(CloudPtrType1 &map_all_pts, CloudPtrType2 &global_undistorted_cloud_waiting_to_add_to_map, CloudPtrType3 &global_undistorted_cloud_smoothing, CloudPtrType4 &curr_undistorted_cloud)
    {
        refrash();

        PublishCloudGE("map_all_pts", map_all_pts);
        PublishCloudGE("global_undistorted_cloud_waiting_to_add_to_map", global_undistorted_cloud_waiting_to_add_to_map);
        PublishCloudGE("global_undistorted_cloud_smoothing", global_undistorted_cloud_smoothing);
        PublishCloudGE("curr_undistorted_cloud", curr_undistorted_cloud);

        static int skip = 20;
        if (skip++ == 20) {
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "camera_init";
            for (auto &pose : path_deque_) {
                path.poses.push_back(pose);
            }
            PublishPath("path", path);
            skip = 0;
        }
    }

    template <typename CloudPtrType1> void PubAll(CloudPtrType1 &map_all_pts)
    {
        refrash();

        PublishCloudGE("map_all_pts", map_all_pts);

        static int skip = 20;
        if (skip++ == 20) {
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "camera_init";
            for (auto &pose : path_deque_) {
                path.poses.push_back(pose);
            }
            PublishPath("path", path);
            skip = 0;
        }
    }

    template <typename CloudPtrType1, typename CloudPtrType2> void PubAll(CloudPtrType1 &map_all_pts, CloudPtrType2 &pt_predict)
    {
        refrash();

        PublishCloudGE("map_all_pts", map_all_pts);
        PublishCloudGE("pt_predict", pt_predict);
        static int skip = 20;
        if (skip++ == 20) {
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "camera_init";
            for (auto &pose : path_deque_) {
                path.poses.push_back(pose);
            }
            PublishPath("path", path);
            skip = 0;
        }
    }

    template <typename CloudPtrType1, typename CloudPtrType2> void PubAll(CloudPtrType1 &map_all_pts, CloudPtrType2 &pt_predict, CloudPtrType2 &pt_predict_dis)
    {
        refrash();

        PublishCloudGE("map_all_pts", map_all_pts);
        PublishCloudGE("pt_predict", pt_predict);
        PublishCloudGE("pt_predict_dis", pt_predict_dis);
        static int skip = 20;
        if (skip++ == 20) {
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "camera_init";
            for (auto &pose : path_deque_) {
                path.poses.push_back(pose);
            }
            PublishPath("path", path);
            skip = 0;
        }
    }

    void AddPath(const Eigen::Quaterniond &quat, const Eigen::Vector3d &pos)
    {
        geometry_msgs::PoseStamped msg_body_pose;
        msg_body_pose.pose.position.x = pos[0];
        msg_body_pose.pose.position.y = pos[1];
        msg_body_pose.pose.position.z = pos[2];
        msg_body_pose.pose.orientation.x = quat.x();
        msg_body_pose.pose.orientation.y = quat.y();
        msg_body_pose.pose.orientation.z = quat.z();
        msg_body_pose.pose.orientation.w = quat.w();
        msg_body_pose.header.stamp = ros::Time::now();
        msg_body_pose.header.frame_id = "camera_init";
        path_deque_.push_back(msg_body_pose);
        // while(path_deque_.size() > 2000) {
        //     path_deque_.pop_front();
        // }
    }

    std::deque<geometry_msgs::PoseStamped> path_deque_;
};