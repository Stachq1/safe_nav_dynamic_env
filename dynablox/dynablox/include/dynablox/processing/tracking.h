#ifndef DYNABLOX_PROCESSING_TRACKING_H_
#define DYNABLOX_PROCESSING_TRACKING_H_

#include <boost/circular_buffer.hpp>

#include "dynablox/common/types.h"
#include "dynablox/minimal_ellipsoid/khach.h"

namespace dynablox {

class Tracking {
 public:
  // Config.
  struct Config {
    Config() : min_track_duration(0), max_tracking_distance(1.f), min_obstacle_size(10), dt_(0.1) {}

    // Numbers of frames a cluster needs to be tracked to be considered dynamic.
    int min_track_duration;

    // Maximum distance a cluster may have moved to be considered a track [m].
    float max_tracking_distance;

    // Minimal number of cluster points to classify as obstacle.
    int min_obstacle_size;

    // Time step for velocity prediction.
    double dt_;
  };

  explicit Tracking(const Config& config = Config()) : config_(config),
           Q_(Eigen::MatrixXd::Identity(2, 2)), c_(Eigen::VectorXd::Zero(2)) {}

  /**
   * @brief Track all clusters w.r.t. the previous clsuters. Denote the object
   * level points dynamic in the cloud_info.
   *
   * @param cloud Current lidar point cloud.
   * @param clusters Current detected clusters.
   * @param cloud_info Cloud info to denote moving points.
   */
  void track(const Cloud& cloud, Clusters& clusters, CloudInfo& cloud_info);

  /**
   * @brief Compute the minimum volume enclosing ellipsoid of a cluster.
   *
   * @param cluster Cluster to compute the MVCE for.
   */
  void computeClusterMVCE(const Cloud& cloud, Cluster& cluster);

 private:
  const Config config_;

  // Tracking data w.r.t. previous observation.
  std::vector<boost::circular_buffer<voxblox::Point>> previous_centroids_;
  std::vector<int> previous_ids_;
  std::vector<int> previous_track_lengths_;

  // Khachiyan ellipsoid algorithm helpers
  MTT<double> Q_;          // Output for the ellipsoid shape matrix
  VTT<double> c_;          // Output for the ellipsoid center
  double eps_ = 1e-1;      // Desired convergence precision
  size_t maxiter_ = 10;    // Maximum number of iterations

  /**
   * @brief Compute centroids of the clusters
   *
   * @param cloud Lidar point cloud.
   * @param clusters Clusters which centers we find.
   * @param centroids Vector to store the centroids.
   */
  void computeCentroids(const Cloud& cloud, Clusters& clusters, std::vector<voxblox::Point>& centroids);

  /**
   * @brief Simple closest association tracking for now.
   *
   * @param cloud Lidar point cloud.
   * @param clusters Current clusters to be tracked.
   */
  void trackClusterIDs(const Cloud& cloud, Clusters& clusters);

  /**
   * @brief Estimate current cluster velocity based on previous cluster positions.
   *
   * @param previous_poses Circular buffer of previous cluster centroid positions.
   */
  voxblox::Point predictClusterVelocity(const boost::circular_buffer<voxblox::Point>& previous_poses);
};

}  // namespace dynablox

#endif  // DYNABLOX_PROCESSING_TRACKING_H_
