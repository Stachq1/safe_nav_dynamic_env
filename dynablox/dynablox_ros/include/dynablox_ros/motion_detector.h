#ifndef DYNABLOX_ROS_MOTION_DETECTOR_H_
#define DYNABLOX_ROS_MOTION_DETECTOR_H_

#include <deque>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>
#include <voxblox_ros/tsdf_server.h>

#include "dynablox/common/index_getter.h"
#include "dynablox/common/types.h"
#include "dynablox/evaluation/evaluator.h"
#include "dynablox/evaluation/ground_truth_handler.h"
#include "dynablox/processing/clustering.h"
#include "dynablox/processing/ever_free_integrator.h"
#include "dynablox/processing/preprocessing.h"
#include "dynablox/processing/tracking.h"
#include "dynablox_ros/visualization/motion_visualizer.h"

namespace dynablox {

class MotionDetector {
 public:
  // Config.
  struct Config {
    Config() : evaluate(false), visualize(true), verbose(true), global_frame_name("map"), sensor_frame_name(""),
               pointcloud_topic("/pointcloud"), queue_size(1), num_threads(std::thread::hardware_concurrency()), shutdown_after(0) {}

    // If true evaluate the performance against GT.
    bool evaluate;

    // Enable helper and debug visualizations.
    bool visualize;

    // Print additional information when running.
    bool verbose;

    // Pointcloud topic name
    std::string pointcloud_topic;

    // Frame names.
    std::string global_frame_name;
    std::string sensor_frame_name;  // Takes msg header if empty, overrides msg header if set.

    // Subscriber queue size.
    int queue_size;

    // Number of threads to use.
    int num_threads;

    // If >0, shutdown after this many evaluated frames.
    int shutdown_after;
  };

  // Constructor.
  MotionDetector(const rclcpp::Node::SharedPtr& nh, const Config& config = Config());

  // Setup.
  void setupMembers();
  void setupRos();

  // Callbacks.
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

  // Motion detection pipeline.
  bool lookupTransform(const std::string& target_frame,
                       const std::string& source_frame, uint64_t timestamp,
                       geometry_msgs::msg::TransformStamped& result) const;

  /**
   * @brief Create a mapping of each voxel index to the points it contains. Each
   * point will be checked whether it falls into an ever-free voxel and updates
   * voxel occupancy, since we go through voxels anyways already.
   *
   * @param cloud Complete point cloud to look up positions.
   * @param point_map Resulting map.
   * @param occupied_ever_free_voxel_indices Indices of voxels containing
   * ever-free points.
   * @param cloud_info Cloud info to store ever-free flags of checked points.
   */
  void setUpPointMap(
      const Cloud& cloud, BlockToPointMap& point_map,
      std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
      CloudInfo& cloud_info) const;

  /**
   * @brief Create a mapping of each block to ids of points that fall into it.
   *
   * @param cloud Points to process.
   * @return Mapping of block to point ids in cloud.
   */
  voxblox::HierarchicalIndexIntMap buildBlockToPointsMap(
      const Cloud& cloud) const;

  /**
   * @brief Create a mapping of each voxel index to the points it contains. Each
   * point will be checked whether it falls into an ever-free voxel and updates
   * voxel occupancy, since we go through voxels anyways already. This function
   * operates on a single block for data parallelism.
   *
   * @param cloud Complete point cloud to look up positions.
   * @param block_index Index of the block to be processed.
   * @param points_in_block Indices of all points in the block.
   * @param point_map Where to store the resulting point map for this block.
   * @param occupied_ever_free_voxel_indices Where to store the indices of ever
   * free voxels in this block.
   * @param cloud_info Cloud info to store ever-free flags of checked points.
   */
  void blockwiseBuildPointMap(
      const Cloud& cloud, const BlockIndex& block_index,
      const voxblox::AlignedVector<size_t>& points_in_block,
      VoxelToPointMap& point_map,
      std::vector<voxblox::VoxelKey>& occupied_ever_free_voxel_indices,
      CloudInfo& cloud_info) const;

 private:
  Eigen::Matrix4f transformStampedToMatrix(
    const geometry_msgs::msg::TransformStamped& transform_stamped);

  const Config config_;

  // ROS.
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pcl_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Voxblox map.
  std::shared_ptr<voxblox::TsdfServer> tsdf_server_;
  std::shared_ptr<TsdfLayer> tsdf_layer_;

  // Configs
  Preprocessing::Config preprocessing_config_;
  EverFreeIntegrator::Config ever_free_integrator_config_;
  Clustering::Config clustering_config_;
  Tracking::Config tracking_config_;
  MotionVisualizer::Config visualizer_config_;

  // Processing.
  std::shared_ptr<Preprocessing> preprocessing_;
  std::shared_ptr<EverFreeIntegrator> ever_free_integrator_;
  std::shared_ptr<Clustering> clustering_;
  std::shared_ptr<Tracking> tracking_;
  std::shared_ptr<Evaluator> evaluator_;
  std::shared_ptr<MotionVisualizer> visualizer_;

  // Cached data.
  size_t voxels_per_side_;
  size_t voxels_per_block_;

  // Variables.
  int frame_counter_ = 0;
};

}  // namespace dynablox

#endif  // DYNABLOX_ROS_MOTION_DETECTOR_H_
