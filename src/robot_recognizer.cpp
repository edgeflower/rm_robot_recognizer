#include "rm_robot_recognizer/robot_recognizer.hpp"
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/detail/pose2_d__struct.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <vector>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

namespace rm_robot_recognizer {
RobotRecognizer::RobotRecognizer()
    : Node("robot_recognizer")
{

    this->declare_parameter<double>("robot_min_size", 0.3);
    this->declare_parameter<double>("robot_max_size", 0.7);
    this->declare_parameter<float>("voxel_leaf", 0.02);
    this->declare_parameter<int>("intensity_threshold", 0);
    this->declare_parameter<double>("cluster_tol", 0.2);
    this->declare_parameter<int>("min_cluster_points", 20);
    this->declare_parameter<int>("max_cluster_points", 5000);
    this->declare_parameter<std::string>("cloud_topic", "terrain_map");
    this->declare_parameter<std::string>("odometry_topic", "odometry");
    this->declare_parameter<std::string>("pub_marker_topic", "robot_markers");
    this->declare_parameter<std::string>("pub_pose2d_topic", "relative_pose2d");
    this->declare_parameter<std::string>("pub_target_topic", "target_pose2d");

    this->get_parameter("robot_min_size", robot_min_size_);
    this->get_parameter("robot_max_size", robot_max_size_);
    this->get_parameter("voxel_leaf", voxel_leaf_);
    this->get_parameter("cluster_tol", cluster_tol_);
    this->get_parameter("min_cluster_points", min_cluster_points_);
    this->get_parameter("max_cluster_points", max_cluster_points_);
    this->get_parameter("intensity_threshold", intensity_threshold_);
    this->get_parameter("cloud_topic", cloud_topic);
    this->get_parameter("odometry_topic", odometry_topic);
    this->get_parameter("pub_marker_topic", pub_marker_topic);
    this->get_parameter("pub_pose2d_topic", pub_pose2d_topic);
    this->get_parameter("pub_target_topic", pub_target_topic);

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, 10,
        std::bind(&RobotRecognizer::cloudCallback, this, std::placeholders::_1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic, 10,
        std::bind(&RobotRecognizer::odomCallback, this, std::placeholders::_1));
    // 发布可视化Maker
//    pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(pub_marker_topic, 1);
    // 发布相对位姿
    pub_pose2d_ = this->create_publisher<geometry_msgs::msg::Pose2D>(pub_pose2d_topic, 1);
    // 发布map坐标
    pub_target_ = this->create_publisher<geometry_msgs::msg::Pose2D>(pub_target_topic, 1);

    RCLCPP_INFO(this->get_logger(), "RobotRecognizer 初始化完成，尺寸范围[%.2f, %.2f]m, 强度阈值%d",
        robot_min_size_, robot_max_size_, intensity_threshold_);
}

void RobotRecognizer::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr last_msg)
{
    // 转换ROS点云PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*last_msg, *cloud);

    // 体素滤波降采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
    vg.filter(*cloud_filt);

    // 强度过滤
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_int(new pcl::PointCloud<pcl::PointXYZI>());
    for (auto& pt : cloud_filt->points) {
        //    if (pt.intensity >= intensity_threshold_) {
        cloud_int->points.push_back(pt);
        //    }
    }
    if (cloud_int->empty())
        return;

    // 欧式聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    tree->setInputCloud(cloud_int);
    std::vector<pcl::PointIndices> clusters; // 簇
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tol_);
    ec.setMinClusterSize(min_cluster_points_);
    ec.setMaxClusterSize(max_cluster_points_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_int);
    ec.extract(clusters);

 //   visualization_msgs::msg::MarkerArray markers;
    // int id = 0;

    for (auto& cluster : clusters) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr part(new pcl::PointCloud<pcl::PointXYZI>());
        for (auto idx : cluster.indices)
            part->points.push_back(cloud_int->points[idx]);

        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*part, min_pt, max_pt);
        float dx = max_pt.x - min_pt.x;
        float dy = max_pt.y - min_pt.y;
        float dz = max_pt.z - min_pt.z;

        if (dx >= robot_min_size_ && dx <= robot_max_size_ && dy >= robot_min_size_ && dy <= robot_max_size_ && dz >= robot_min_size_ && dz <= robot_max_size_) {
            RCLCPP_INFO(this->get_logger(),
                "检测到机器人簇：尺寸(%.2f, %.2f, %.2f)", dx, dy, dz);
            float cx = (min_pt.x + max_pt.x) * 0.5f;
            float cy = (min_pt.y + max_pt.y) * 0.5f;
            double theta = std::atan2(cy, cx); // 角度计算
            {
                std::lock_guard<std::mutex> lk(mutex_);
                last_rel_x_ = cx;
                last_rel_y_ = cy;
            }
            geometry_msgs::msg::Pose2D pose2d;
            pose2d.x = cx;
            pose2d.y = cy;
            pose2d.theta = theta;
            pub_pose2d_->publish(pose2d);
            RCLCPP_INFO(this->get_logger(), "相对位置[%.2f,%.2f]", pose2d.x, pose2d.y);
            /*
                        {visualization_msgs::msg::Marker m;
                        m.header = last_msg->header;
                        m.ns = "robot_recognizer";
                        m.id = id++;
                        m.type = visualization_msgs::msg::Marker::CUBE;
                        m.action = visualization_msgs::msg::Marker::ADD;
                        m.scale.x = dx;
                        m.scale.y = dy;
                        m.scale.z = dz;
                        m.pose.position.x = cx;
                        m.pose.position.y = cy;
                        m.pose.position.z = dz * 0.5f;
                        m.pose.orientation.w = 1.0;
                        m.color.g = 1.0;
                        m.color.a = 0.8;
                        markers.markers.push_back(m);
                        }
                        */
        }
    }
//    pub_marker_->publish(markers);
}

void RobotRecognizer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr last_msg)
{
    float rx, ry;
    {
        std::lock_guard<std::mutex> lk(mutex_);

        rx = last_rel_x_;
        ry = last_rel_y_;
    }

    last_odom_x_ = last_msg->pose.pose.position.x;
    last_odom_y_ = last_msg->pose.pose.position.y;
    // RCLCPP_INFO(this->get_logger(), "自己位置[%.2f,%.2f]", last_odom_x_, last_odom_y_); // Debug
    target_pose2d.x = last_odom_x_ + rx;
    target_pose2d.y = last_odom_y_ + ry;
    if (target_pose2d.x != last_odom_x_ && target_pose2d.y != last_odom_y_) {
        pub_target_->publish(target_pose2d);
        RCLCPP_INFO(this->get_logger(), "全局位置 [%.2f,%.2f]", target_pose2d.x, target_pose2d.y);
    }
    // 置零,避免未识别到输出错误位置
    {
        std::lock_guard<std::mutex> lk(mutex_);
        last_rel_x_ = 0.0;
        last_rel_y_ = 0.0;
    }
}

} // namespace rm_robot_recognizer

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_robot_recognizer::RobotRecognizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
