#include "global_pose_initializer/global_pose_initializer.hpp"

GlobalPoseInitializer::GlobalPoseInitializer(const rclcpp::NodeOptions & node_options)
: Node("global_pose_initializer", node_options)
{
  declare_parameter<double>("normal_estimation.search_radius");
  declare_parameter<double>("fpfh.search_radius");
  declare_parameter<std::string>("fpfh_map_path");
  declare_parameter<bool>("load_fpfh_feature");
  get_parameter<bool>("load_fpfh_feature", load_fpfh_feature_);
  declare_parameter<double>("map_downsample_leaf_size");
  declare_parameter<double>("sensor_downsample_leaf_size");

  sensor_points_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", 5,
    std::bind(&GlobalPoseInitializer::callback_sensor_points, this, std::placeholders::_1));
  global_map_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&GlobalPoseInitializer::callback_global_map, this, std::placeholders::_1));

  pose_publisher_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 5);

  pose_initialize_srv_ = create_service<std_srvs::srv::Empty>(
    "global_initialize",
    std::bind(&GlobalPoseInitializer::process, this, std::placeholders::_1, std::placeholders::_2));
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr GlobalPoseInitializer::extract_fpfh(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  RCLCPP_INFO_STREAM(get_logger(), "extract fpfh");
  double normal_estimation_radius;
  get_parameter<double>("normal_estimation.search_radius", normal_estimation_radius);
  double fpfh_search_radius;
  get_parameter<double>("fpfh.search_radius", fpfh_search_radius);

  RCLCPP_INFO_STREAM(get_logger(), "Normal Estimation");
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
  normal_estimation.setRadiusSearch(normal_estimation_radius);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  normal_estimation.setSearchMethod(kdtree);
  normal_estimation.setInputCloud(cloud);
  normal_estimation.compute(*normals);

  RCLCPP_INFO_STREAM(get_logger(), "FPFH Estimation");
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
  fpfh_estimation.setRadiusSearch(fpfh_search_radius);
  fpfh_estimation.setInputCloud(cloud);
  fpfh_estimation.setInputNormals(normals);
  fpfh_estimation.compute(*features);

  return features;
}

void GlobalPoseInitializer::downsample(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr input_points_ptr,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & output_points_ptr, float downsample_leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(downsample_leaf_size, downsample_leaf_size, downsample_leaf_size);
  voxel_grid_filter.setInputCloud(input_points_ptr);
  voxel_grid_filter.filter(*output_points_ptr);
}

void GlobalPoseInitializer::process(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  RCLCPP_INFO_STREAM(get_logger(), "global initialize...");

  pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(sensor_points_msg, *sensor_points);

  double sensor_downsample_leaf_size;
  get_parameter<double>("sensor_downsample_leaf_size", sensor_downsample_leaf_size);
  downsample(sensor_points, filtered_points, sensor_downsample_leaf_size);

  teaser::PointCloud source_cloud;
  for (std::size_t i = 0; i < filtered_points->size(); i++) {
    source_cloud.push_back(
      {filtered_points->at(i).x, filtered_points->at(i).y, filtered_points->at(i).z});
  }

  teaser::FPFHCloud::Ptr source_feature = extract_fpfh(filtered_points);

  teaser::Matcher matcher;
  auto correspondences = matcher.calculateCorrespondences(
    source_cloud, target_cloud_, *source_feature, *target_feature_, false, false, false, 0.95);

  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = 0.5;
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 100;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm =
    teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = 0.005;

  RCLCPP_INFO_STREAM(get_logger(), "Solve Regstration");
  teaser::RobustRegistrationSolver solver(params);
  solver.solve(source_cloud, target_cloud_, correspondences);
  RCLCPP_INFO_STREAM(get_logger(), "Finish Solve");

  auto solution = solver.getSolution();
  auto translation = solution.translation;
  auto rotation = solution.rotation;
  Eigen::Quaterniond quaternion(rotation);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.frame_id = "map";
  pose_msg.header.stamp = now();
  pose_msg.pose.pose.position.x = translation.x();
  pose_msg.pose.pose.position.y = translation.y();
  pose_msg.pose.pose.position.z = translation.z();
  pose_msg.pose.pose.orientation.x = quaternion.x();
  pose_msg.pose.pose.orientation.y = quaternion.y();
  pose_msg.pose.pose.orientation.z = quaternion.z();
  pose_msg.pose.pose.orientation.w = quaternion.w();
  pose_publisher_->publish(pose_msg);
}

void GlobalPoseInitializer::callback_global_map(const sensor_msgs::msg::PointCloud2 & msg)
{
  global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msg, *map);

  double map_downsample_leaf_size;
  get_parameter<double>("map_downsample_leaf_size", map_downsample_leaf_size);
  downsample(map, global_map_, map_downsample_leaf_size);

  for (std::size_t i = 0; i < global_map_->size(); i++) {
    target_cloud_.push_back({global_map_->at(i).x, global_map_->at(i).y, global_map_->at(i).z});
  }

  std::string fpfh_map_path;
  get_parameter<std::string>("fpfh_map_path", fpfh_map_path);
  if (load_fpfh_feature_) {
    RCLCPP_INFO_STREAM(get_logger(), "Load FPFH Feature Map.");

    target_feature_.reset(new teaser::FPFHCloud);
    if (pcl::io::loadPCDFile<pcl::FPFHSignature33>(fpfh_map_path, *target_feature_) == -1) {
      RCLCPP_ERROR_STREAM(get_logger(), "Couldn't read file: " << fpfh_map_path.c_str());
      return;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Load Finish.");
  } else {
    target_feature_ = extract_fpfh(global_map_);
    RCLCPP_INFO_STREAM(get_logger(), "output fpfh feature");
    pcl::io::savePCDFile(fpfh_map_path, *target_feature_);
  }
}

void GlobalPoseInitializer::callback_sensor_points(const sensor_msgs::msg::PointCloud2 & msg)
{
  sensor_points_msg = msg;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GlobalPoseInitializer)
