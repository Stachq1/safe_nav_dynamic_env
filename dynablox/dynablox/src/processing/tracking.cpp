#include "dynablox/processing/tracking.h"

namespace dynablox {

void Tracking::track(const Cloud& cloud, Clusters& clusters,
                     CloudInfo& cloud_info) {
  // Associate current to previous cluster ids.
  trackClusterIDs(cloud, clusters);

  // Compute MVCEs, predict cluster velocities and label the cloud info.
  for (Cluster& cluster : clusters) {
    computeClusterMVCE(cloud, cluster);
    cluster.est_velocity = predictClusterVelocity(cluster.previous_centroids);
    if (cluster.track_length >= config_.min_track_duration) {
      cluster.valid = true;
      for (int idx : cluster.points) {
        cloud_info.points[idx].object_level_dynamic = true;
      }
    }
  }
}

void Tracking::computeClusterMVCE(const Cloud& cloud, Cluster& cluster) {
  unsigned int n = cluster.points.size();
  // TODO: What if n is really small? Can that even happen?

  // Create the points input matrix
  Eigen::MatrixXd points(2, n);
  for (unsigned int i = 0; i < n; ++i) {
    const Point& point = cloud[cluster.points[i]];
    points(0, i) = point.x;
    points(1, i) = point.y;
  }

  // Compute the MVCE
  Tracking::Ellipsoid mvce = Tracking::Ellipsoid::MinimumVolumeCircumscribedEllipsoid(points);

  // Save the MVCE matrix A and center into the cluster
  cluster.mvce_A = mvce.A();
  cluster.mvce_center = mvce.center();
}

void Tracking::computeCentroids(const Cloud& cloud, Clusters& clusters, std::vector<voxblox::Point>& centroids) {
  size_t i = 0;
  for (Cluster& cluster : clusters) {
    voxblox::Point centroid = {0, 0, 0};
    for (int index : cluster.points) {
      const Point& point = cloud[index];
      centroid = centroid + voxblox::Point(point.x, point.y, point.z);
    }
    centroids[i] = centroid / cluster.points.size();
    cluster.previous_centroids.push_back(centroids[i]);
    ++i;
  }
}

void Tracking::trackClusterIDs(const Cloud& cloud, Clusters& clusters) {
  // Compute the centroids of all clusters.
  std::vector<voxblox::Point> centroids(clusters.size());
  computeCentroids(cloud, clusters, centroids);

  // Compute the distances of all clusters. [previous][current]->dist
  struct Association {
    float distance;
    int previous_id;
    int current_id;
  };

  std::vector<std::vector<Association>> distances(previous_centroids_.size());
  for (size_t i = 0; i < previous_centroids_.size(); ++i) {
    std::vector<Association>& d = distances[i];
    d.reserve(centroids.size());
    for (size_t j = 0; j < centroids.size(); ++j) {
      Association association;
      association.distance = (previous_centroids_[i] - centroids[j]).norm();
      association.previous_id = i;
      association.current_id = j;
      d.push_back(association);
    }
  }

  // Associate all previous ids until no more minimum distances exist.
  std::unordered_set<int> reused_ids;
  while (true) {
    // Find the minimum distance and IDs (exhaustively).
    float min = std::numeric_limits<float>::max();
    int prev_id = 0;
    int curr_id = 0;
    int erase_i = 0;
    int erase_j = 0;
    for (size_t i = 0u; i < distances.size(); ++i) {
      for (size_t j = 0u; j < distances[i].size(); ++j) {
        const Association& association = distances[i][j];
        if (association.distance < min) {
          min = association.distance;
          curr_id = association.current_id;
          prev_id = association.previous_id;
          erase_i = i;
          erase_j = j;
        }
      }
    }

    if (min > config_.max_tracking_distance) {
      // no more good fits.
      break;
    }

    // Update traked cluster and remove that match to search for next best.
    clusters[curr_id].id = previous_ids_[prev_id];
    clusters[curr_id].track_length = previous_track_lengths_[prev_id] + 1;
    reused_ids.insert(previous_ids_[prev_id]);
    distances.erase(distances.begin() + erase_i);
    for (auto& vec : distances) {
      vec.erase(vec.begin() + erase_j);
    }
  }

  // Fill in all remaining ids and track data.
  previous_centroids_ = centroids;
  previous_ids_.clear();
  previous_ids_.reserve(clusters.size());
  previous_track_lengths_.clear();
  previous_ids_.reserve(clusters.size());

  int id_counter = 0;
  for (Cluster& cluster : clusters) {
    if (cluster.id == -1) {
      // We need to replace it.
      while (reused_ids.find(id_counter) != reused_ids.end()) {
        id_counter++;
      }
      cluster.id = id_counter;
      id_counter++;
    }
    previous_ids_.push_back(cluster.id);
    previous_track_lengths_.push_back(cluster.track_length);
  }
}

voxblox::Point Tracking::predictClusterVelocity(const boost::circular_buffer<voxblox::Point>& previous_poses) {
  unsigned int n = previous_poses.size();
  voxblox::Point est_velocity = voxblox::Point::Zero();

  // Ensure there are at least two points to calculate velocity
  if (n < 2) {
    return est_velocity;
  }

  // Compute average velocity
  for (size_t i = 1; i < n; ++i) {
      est_velocity += (previous_poses[i] - previous_poses[i - 1]);
  }
  est_velocity /= (n - 1);

  return est_velocity;
}

}  // namespace dynablox
