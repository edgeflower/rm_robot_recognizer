#ifndef RM_ROBOT_RECOGNiZER__ROBOT_RECOGNIZER_HPP_
#define RM_ROBOT_RECOGNiZER__ROBOT_RECOGNIZER_HPP_
#include <geometry_msgs/msg/detail/pose2_d__struct.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <mutex>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
namespace rm_robot_recognizer {
class RobotRecognizer : public rclcpp::Node {
public:
    RobotRecognizer();

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr last_msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr last_msg);

    double robot_min_size_;
    double robot_max_size_;
    int min_cluster_points_; // 最小簇点数
    int max_cluster_points_;// 最大簇点数
    double cluster_tol_;  // 聚类容差（米）
    double window_time_; // 时间窗口（秒）
    int window_frame_;  // 帧数窗口
    float voxel_leaf_; // 体素大小 （米）
    int intensity_threshold_; // 反射强度

    std::string cloud_topic;
    std::string odometry_topic;
    //std::string pub_marker_topic;
    std::string pub_pose2d_topic;
    std::string pub_target_topic;

    float last_odom_x_ { 0.0f };
    float last_odom_y_ { 0.0f };
    geometry_msgs::msg::Pose2D target_pose2d;
    std::mutex mutex_;

    // 最新识别得到的相对偏移（base_link 下）
    float last_rel_x_ {0};
    float last_rel_y_ {0};
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    //rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_pose2d_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_target_;
};
}

#endif // RM_ROBOT_RECOGNiZER__ROBOT_RECOGNIZER_HPP_