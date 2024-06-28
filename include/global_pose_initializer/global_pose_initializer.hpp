#ifndef GLOBAL_POSE_INITIALIZER__GLOBAL_POSE_INIITALIZER_HPP_
#define GLOBAL_POSE_INITIALIZER__GLOBAL_POSE_INIITALIZER_HPP_

#include <pcl/search/impl/kdtree.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/gfpfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <teaser/fpfh.h>
#include <teaser/matcher.h>
#include <teaser/registration.h>

class GlobalPoseInitializer : public rclcpp::Node
{
public:
  explicit GlobalPoseInitializer(const rclcpp::NodeOptions & node_options);
  ~GlobalPoseInitializer() = default;

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr extract_fpfh(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  void callback_global_map(const sensor_msgs::msg::PointCloud2 & msg);
  void callback_sensor_points(const sensor_msgs::msg::PointCloud2 & msg);

  void process(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void downsample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_points_ptr,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & output_points_ptr, float downsample_leaf_size);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pose_initialize_srv_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  sensor_msgs::msg::PointCloud2 sensor_points_msg;

  teaser::PointCloud target_cloud_;
  teaser::FPFHCloud::Ptr target_feature_;

  bool load_fpfh_feature_;
};

#endif
