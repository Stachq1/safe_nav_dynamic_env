#ifndef DYNABLOX_PROCESSING_TRACKING_H_
#define DYNABLOX_PROCESSING_TRACKING_H_

#include "dynablox/common/types.h"

namespace dynablox {

class Tracking {
 public:
  // Config.
  struct Config {
    Config() : min_track_duration(0), max_tracking_distance(1.f) {}

    // Numbers of frames a cluster needs to be tracked to be considered dynamic.
    int min_track_duration;

    // Maximum distance a cluster may have moved to be considered a track [m].
    float max_tracking_distance;
  };

  explicit Tracking(const Config& config = Config()) : config_(config) {}

  /**
   * @brief Track all clusters w.r.t. the previous clsuters. Denote the object
   * level points dynamic in the cloud_info.
   *
   * @param cloud Current lidar point cloud.
   * @param clusters Current detected clusters.
   * @param cloud_info Cloud info to denote moving points.
   */
  void track(const Cloud& cloud, Clusters& clusters, CloudInfo& cloud_info);

 private:
  const Config config_;

  // Tracking data w.r.t. previous observation.
  std::vector<voxblox::Point> previous_centroids_;
  std::vector<int> previous_ids_;
  std::vector<int> previous_track_lengths_;

  /**
   * @brief Compute centroids of the clusters
   *
   * @param cloud Lidar point cloud.
   * @param clusters Clusters which centers we find.
   */
  std::vector<voxblox::Point> computeCentroids(const Cloud& cloud, Clusters& clusters);

  /**
   * @brief Simple closest association tracking for now.
   *
   * @param cloud Lidar point cloud.
   * @param clusters Current clusters to be tracked.
   */
  void trackClusterIDs(const Cloud& cloud, Clusters& clusters);

  /**
   * @brief Predict the next centroid of a cluster based on previous positions.
   *
   * @param previous_poses Circular buffer of previous cluster centroid positions.
   */
  voxblox::Point predictNextPosition(const boost::circular_buffer<voxblox::Point>& previous_poses);

};

}  // namespace dynablox

#endif  // DYNABLOX_PROCESSING_TRACKING_H_
