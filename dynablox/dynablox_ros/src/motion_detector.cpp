#include "dynablox_ros/motion_detector.h"

#include <math.h>

#include <future>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

#include <minkindr/kindr_msg.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace dynablox {

using Timer = voxblox::timing::Timer;

MotionDetector::MotionDetector(const rclcpp::Node::SharedPtr& nh, const Config& config)
    : config_(config),
      nh_(nh),
      tf_buffer_(nh_->get_clock()),
      tf_listener_(tf_buffer_) {

  // Subscribe to topics and load parameters from config
  setupRos();

  // Setup Dynablox modules
  setupMembers();

  // Cache frequently used constants.
  voxels_per_side_ = tsdf_layer_->voxels_per_side();
  voxels_per_block_ = voxels_per_side_ * voxels_per_side_ * voxels_per_side_;
}

void MotionDetector::setupMembers() {
// Create a new node for voxblox parameters
  auto nh_voxblox = std::make_shared<rclcpp::Node>("voxblox", nh_->get_namespace());

  // Initialize the TSDF server with parameters from the new voxblox node.
  tsdf_server_ = std::make_shared<voxblox::TsdfServer>(nh_voxblox, nh_voxblox);
  tsdf_layer_.reset(tsdf_server_->getTsdfMapPtr()->getTsdfLayerPtr());

  // Initialize Dynablox modules.
  preprocessing_ = std::make_shared<Preprocessing>(preprocessing_config_);
  clustering_ = std::make_shared<Clustering>(tsdf_layer_, clustering_config_);
  tracking_ = std::make_shared<Tracking>(tracking_config_);
  ever_free_integrator_ = std::make_shared<EverFreeIntegrator>(tsdf_layer_, ever_free_integrator_config_);

  // Evaluation.
  if (config_.evaluate) {
    // Initialize evaluator if evaluation is requested.
    evaluator_ = std::make_shared<Evaluator>();
  }

  // Visualization.
  visualizer_ = std::make_shared<MotionVisualizer>(nh_, tsdf_layer_, visualizer_config_);
}

void MotionDetector::setupRos() {
  // Preprocessing config
  nh_->declare_parameter("preprocessing.max_range", preprocessing_config_.max_range);
  nh_->declare_parameter("preprocessing.min_range", preprocessing_config_.min_range);
  nh_->get_parameter("preprocessing.max_range", preprocessing_config_.max_range);
  nh_->get_parameter("preprocessing.max_range", preprocessing_config_.min_range);

  // Ever-Free Integrator config
  nh_->declare_parameter("ever_free_integrator.neighbor_connectivity", ever_free_integrator_config_.neighbor_connectivity);
  nh_->declare_parameter("ever_free_integrator.counter_to_reset", ever_free_integrator_config_.counter_to_reset);
  nh_->declare_parameter("ever_free_integrator.temporal_buffer", ever_free_integrator_config_.temporal_buffer);
  nh_->declare_parameter("ever_free_integrator.burn_in_period", ever_free_integrator_config_.burn_in_period);
  nh_->declare_parameter("ever_free_integrator.tsdf_occupancy_threshold", ever_free_integrator_config_.tsdf_occupancy_threshold);
  nh_->declare_parameter("ever_free_integrator.num_threads", ever_free_integrator_config_.num_threads);
  nh_->get_parameter("ever_free_integrator.neighbor_connectivity", ever_free_integrator_config_.neighbor_connectivity);
  nh_->get_parameter("ever_free_integrator.counter_to_reset", ever_free_integrator_config_.counter_to_reset);
  nh_->get_parameter("ever_free_integrator.temporal_buffer", ever_free_integrator_config_.temporal_buffer);
  nh_->get_parameter("ever_free_integrator.burn_in_period", ever_free_integrator_config_.burn_in_period);
  nh_->get_parameter("ever_free_integrator.tsdf_occupancy_threshold", ever_free_integrator_config_.tsdf_occupancy_threshold);
  nh_->get_parameter("ever_free_integrator.num_threads", ever_free_integrator_config_.num_threads);

  // Clustering config
  nh_->declare_parameter("clustering.min_cluster_size", clustering_config_.min_cluster_size);
  nh_->declare_parameter("clustering.max_cluster_size", clustering_config_.max_cluster_size);
  nh_->declare_parameter("clustering.min_extent", clustering_config_.min_extent);
  nh_->declare_parameter("clustering.neighbor_connectivity", clustering_config_.neighbor_connectivity);
  nh_->declare_parameter("clustering.grow_clusters_twice", clustering_config_.grow_clusters_twice);
  nh_->declare_parameter("clustering.min_cluster_separation", clustering_config_.min_cluster_separation);
  nh_->declare_parameter("clustering.check_cluster_separation_exact", clustering_config_.check_cluster_separation_exact);
  nh_->get_parameter("clustering.min_cluster_size", clustering_config_.min_cluster_size);
  nh_->get_parameter("clustering.max_cluster_size", clustering_config_.max_cluster_size);
  nh_->get_parameter("clustering.min_extent", clustering_config_.min_extent);
  nh_->get_parameter("clustering.neighbor_connectivity", clustering_config_.neighbor_connectivity);
  nh_->get_parameter("clustering.grow_clusters_twice", clustering_config_.grow_clusters_twice);
  nh_->get_parameter("clustering.min_cluster_separation", clustering_config_.min_cluster_separation);
  nh_->get_parameter("clustering.check_cluster_separation_exact", clustering_config_.check_cluster_separation_exact);

  // Tracking config
  nh_->declare_parameter("tracking.min_track_duration", tracking_config_.min_track_duration);
  nh_->declare_parameter("tracking.max_tracking_distance", tracking_config_.max_tracking_distance);
  nh_->get_parameter("tracking.min_track_duration", tracking_config_.min_track_duration);
  nh_->get_parameter("tracking.max_tracking_distance", tracking_config_.max_tracking_distance);

  // Visualization config
  nh_->declare_parameter("visualization.static_point_color", visualizer_config_.static_point_color);
  nh_->declare_parameter("visualization.dynamic_point_color", visualizer_config_.dynamic_point_color);
  nh_->declare_parameter("visualization.out_of_bounds_color", visualizer_config_.out_of_bounds_color);
  nh_->declare_parameter("visualization.ever_free_color", visualizer_config_.ever_free_color);
  nh_->declare_parameter("visualization.never_free_color", visualizer_config_.never_free_color);
  nh_->declare_parameter("visualization.static_point_scale", visualizer_config_.static_point_scale);
  nh_->declare_parameter("visualization.dynamic_point_scale", visualizer_config_.dynamic_point_scale);
  nh_->declare_parameter("visualization.sensor_scale", visualizer_config_.sensor_scale);
  nh_->declare_parameter("visualization.color_clusters", visualizer_config_.color_clusters);
  nh_->declare_parameter("visualization.color_wheel_num_colors", visualizer_config_.color_wheel_num_colors);
  nh_->declare_parameter("visualization.slice_height", visualizer_config_.slice_height);
  nh_->declare_parameter("visualization.visualization_max_z", visualizer_config_.visualization_max_z);
  nh_->get_parameter("visualization.static_point_color", visualizer_config_.static_point_color);
  nh_->get_parameter("visualization.dynamic_point_color", visualizer_config_.dynamic_point_color);
  nh_->get_parameter("visualization.out_of_bounds_color", visualizer_config_.out_of_bounds_color);
  nh_->get_parameter("visualization.ever_free_color", visualizer_config_.ever_free_color);
  nh_->get_parameter("visualization.never_free_color", visualizer_config_.never_free_color);
  nh_->get_parameter("visualization.static_point_scale", visualizer_config_.static_point_scale);
  nh_->get_parameter("visualization.dynamic_point_scale", visualizer_config_.dynamic_point_scale);
  nh_->get_parameter("visualization.sensor_scale", visualizer_config_.sensor_scale);
  nh_->get_parameter("visualization.color_clusters", visualizer_config_.color_clusters);
  nh_->get_parameter("visualization.color_wheel_num_colors", visualizer_config_.color_wheel_num_colors);
  nh_->get_parameter("visualization.slice_height", visualizer_config_.slice_height);
  nh_->get_parameter("visualization.visualization_max_z", visualizer_config_.visualization_max_z);

  // Set visualization frame to be the same as the global frame
  visualizer_config_.global_frame_name = config_.global_frame_name;

  // Subscribe to pointcloud topic
  lidar_pcl_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
    config_.pointcloud_topic, rclcpp::QoS(config_.queue_size),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        this->pointcloudCallback(msg);
    });
}

void MotionDetector::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  Timer frame_timer("frame");
  Timer detection_timer("motion_detection");

  // Lookup cloud transform T_M_S of sensor (S) to map (M).
  // If different sensor frame is required, update the message.
  Timer tf_lookup_timer("motion_detection/tf_lookup");
  const std::string sensor_frame_name = config_.sensor_frame_name.empty()
                                            ? msg->header.frame_id
                                            : config_.sensor_frame_name;

  geometry_msgs::msg::TransformStamped T_M_S;
  if (!lookupTransform(config_.global_frame_name, sensor_frame_name,
                       rclcpp::Time(msg->header.stamp).nanoseconds(), T_M_S)) {
    // Getting transform failed, need to skip.
    return;
  }
  tf_lookup_timer.Stop();

  // Preprocessing.
  Timer preprocessing_timer("motion_detection/preprocessing");
  frame_counter_++;
  CloudInfo cloud_info;
  Cloud cloud;

  Eigen::Matrix4f T_M_S_4f = transformStampedToMatrix(T_M_S);
  preprocessing_->processPointcloud(msg, T_M_S_4f, cloud, cloud_info, rclcpp::Time(msg->header.stamp).nanoseconds()); // What do I do here?
  preprocessing_timer.Stop();

  // Build a mapping of all blocks to voxels to points for the scan.
  Timer setup_timer("motion_detection/indexing_setup");
  BlockToPointMap point_map;
  std::vector<voxblox::VoxelKey> occupied_ever_free_voxel_indices;
  setUpPointMap(cloud, point_map, occupied_ever_free_voxel_indices, cloud_info);
  setup_timer.Stop();

  // Clustering.
  Timer clustering_timer("motion_detection/clustering");
  Clusters clusters = clustering_->performClustering(
      point_map, occupied_ever_free_voxel_indices, frame_counter_, cloud,
      cloud_info);
  clustering_timer.Stop();

  // Tracking.
  Timer tracking_timer("motion_detection/tracking");
  tracking_->track(cloud, clusters, cloud_info);
  tracking_timer.Stop();

  // Integrate ever-free information.
  Timer update_ever_free_timer("motion_detection/update_ever_free");
  ever_free_integrator_->updateEverFreeVoxels(frame_counter_);
  update_ever_free_timer.Stop();

  // Integrate the pointcloud into the voxblox TSDF map.
  Timer tsdf_timer("motion_detection/tsdf_integration");
  voxblox::Transformation T_G_C;
  tf2::transformMsgToKindr(T_M_S.transform, &T_G_C);
  tsdf_server_->processPointCloudMessageAndInsert(msg, T_G_C, false);
  tsdf_timer.Stop();
  detection_timer.Stop();

  // Evaluation if requested.
  if (config_.evaluate) {
    Timer eval_timer("evaluation");
    evaluator_->evaluateFrame(cloud, cloud_info, clusters);
    eval_timer.Stop();
    if (config_.shutdown_after > 0 &&
        evaluator_->getNumberOfEvaluatedFrames() >= config_.shutdown_after) {
      LOG(INFO) << "Evaluated " << config_.shutdown_after
                << " frames, shutting down";
      rclcpp::shutdown();
    }
  }

  // Visualization if requested.
  if (config_.visualize) {
    Timer vis_timer("visualizations");
    visualizer_->visualizeAll(cloud, cloud_info, clusters);
    vis_timer.Stop();
  }
}

bool MotionDetector::lookupTransform(const std::string& target_frame,
                                     const std::string& source_frame,
                                     uint64_t timestamp,
                                     geometry_msgs::msg::TransformStamped& result) const {
  // Convert the timestamp to rclcpp::Time
  rclcpp::Time timestamp_ros(timestamp);

  try {
    // Use the lookupTransform method from the tf_buffer
    result = tf_buffer_.lookupTransform(target_frame, source_frame, timestamp_ros);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(nh_->get_logger(), "Could not get sensor transform, skipping pointcloud: %s", ex.what());
    return false;
  }
  return true;
}

void MotionDetector::setUpPointMap(
    const Cloud& cloud, BlockToPointMap& point_map,
    std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
    CloudInfo& cloud_info) const {
  // Identifies for any LiDAR point the block it falls in and constructs the
  // hash-map block2points_map mapping each block to the LiDAR points that
  // fall into the block.
  const voxblox::HierarchicalIndexIntMap block2points_map =
      buildBlockToPointsMap(cloud);

  // Builds the voxel2point-map in parallel blockwise.
  std::vector<BlockIndex> block_indices(block2points_map.size());
  size_t i = 0;
  for (const auto& block : block2points_map) {
    block_indices[i] = block.first;
    ++i;
  }
  IndexGetter<BlockIndex> index_getter(block_indices);
  std::vector<std::future<void>> threads;
  std::mutex aggregate_results_mutex;
  for (int i = 0; i < config_.num_threads; ++i) {
    threads.emplace_back(std::async(std::launch::async, [&]() {
      // Data to store results.
      BlockIndex block_index;
      std::vector<voxblox::VoxelKey> local_occupied_indices;
      BlockToPointMap local_point_map;

      // Process until no more blocks.
      while (index_getter.getNextIndex(&block_index)) {
        VoxelToPointMap result;
        this->blockwiseBuildPointMap(cloud, block_index,
                                     block2points_map.at(block_index), result,
                                     local_occupied_indices, cloud_info);
        local_point_map.insert(std::pair(block_index, result));
      }

      // After processing is done add data to the output map.
      std::lock_guard<std::mutex> lock(aggregate_results_mutex);
      occupied_ever_free_voxel_indices.insert(
          occupied_ever_free_voxel_indices.end(),
          local_occupied_indices.begin(), local_occupied_indices.end());
      point_map.merge(local_point_map);
    }));
  }

  for (auto& thread : threads) {
    thread.get();
  }
}

voxblox::HierarchicalIndexIntMap MotionDetector::buildBlockToPointsMap(
    const Cloud& cloud) const {
  voxblox::HierarchicalIndexIntMap result;

  int i = 0;
  for (const Point& point : cloud) {
    voxblox::Point coord(point.x, point.y, point.z);
    const BlockIndex blockindex =
        tsdf_layer_->computeBlockIndexFromCoordinates(coord);
    result[blockindex].push_back(i);
    i++;
  }
  return result;
}

void MotionDetector::blockwiseBuildPointMap(
    const Cloud& cloud, const BlockIndex& block_index,
    const voxblox::AlignedVector<size_t>& points_in_block,
    VoxelToPointMap& voxel_map,
    std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
    CloudInfo& cloud_info) const {
  // Get the block.
  TsdfBlock::Ptr tsdf_block = tsdf_layer_->getBlockPtrByIndex(block_index);
  if (!tsdf_block) {
    return;
  }

  // Create a mapping of each voxel index to the points it contains.
  for (size_t i : points_in_block) {
    const Point& point = cloud[i];
    const voxblox::Point coords(point.x, point.y, point.z);
    const VoxelIndex voxel_index =
        tsdf_block->computeVoxelIndexFromCoordinates(coords);
    if (!tsdf_block->isValidVoxelIndex(voxel_index)) {
      continue;
    }
    voxel_map[voxel_index].push_back(i);

    // EverFree detection flag at the same time, since we anyways lookup
    // voxels.
    if (tsdf_block->getVoxelByVoxelIndex(voxel_index).ever_free) {
      cloud_info.points.at(i).ever_free_level_dynamic = true;
    }
  }

  // Update the voxel status of the currently occupied voxels.
  for (const auto& voxel_points_pair : voxel_map) {
    TsdfVoxel& tsdf_voxel =
        tsdf_block->getVoxelByVoxelIndex(voxel_points_pair.first);
    tsdf_voxel.last_lidar_occupied = frame_counter_;

    // This voxel attribute is used in the voxel clustering method: it
    // signalizes that a currently occupied voxel has not yet been clustered
    tsdf_voxel.clustering_processed = false;

    // The set of occupied_ever_free_voxel_indices allows for fast access of
    // the seed voxels in the voxel clustering
    if (tsdf_voxel.ever_free) {
      occupied_ever_free_voxel_indices.push_back(
          std::make_pair(block_index, voxel_points_pair.first));
    }
  }
}

Eigen::Matrix4f MotionDetector::transformStampedToMatrix(const geometry_msgs::msg::TransformStamped& transform_stamped) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Extract translation
  transform(0, 3) = transform_stamped.transform.translation.x;
  transform(1, 3) = transform_stamped.transform.translation.y;
  transform(2, 3) = transform_stamped.transform.translation.z;

  // Extract rotation and convert to Eigen quaternion
  Eigen::Quaternionf q(
      transform_stamped.transform.rotation.w,
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z
  );

  // Convert quaternion to rotation matrix and place it in the top-left 3x3 part
  transform.block<3, 3>(0, 0) = q.toRotationMatrix();

  return transform;
}

}  // namespace dynablox
