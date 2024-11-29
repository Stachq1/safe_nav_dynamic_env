#include "dynablox/processing/tracking.h"

namespace dynablox {

void Tracking::track(const Cloud& cloud, Clusters& clusters,
                     CloudInfo& cloud_info) {
  // Associate current to previous cluster ids.
  trackClusterIDs(cloud, clusters);

  // Label the cloud info.
  for (Cluster& cluster : clusters) {
    if (cluster.track_length >= config_.min_track_duration) {
      cluster.valid = true;
      for (int idx : cluster.points) {
        cloud_info.points[idx].object_level_dynamic = true;
      }
    }
  }
}

std::vector<voxblox::Point> Tracking::computeCentroids(const Cloud& cloud, Clusters& clusters) {
  std::vector<voxblox::Point> centroids(clusters.size());
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

  return centroids;
}

void Tracking::trackClusterIDs(const Cloud& cloud, Clusters& clusters) {
  // Compute the centroids of all clusters.
  std::vector<voxblox::Point> centroids = computeCentroids(cloud, clusters);

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

voxblox::Point predictNextPosition(const boost::circular_buffer<voxblox::Point>& previous_poses) {
  unsigned int n = previous_poses.size();

  // Ensure there are at least two points to calculate velocity
  if (n < 2) {
    return previous_poses.back();
  }

  voxblox::Point velocity = voxblox::Point::Zero();

  // Compute average velocity
  for (size_t i = 1; i < n; ++i) {
      velocity += (previous_poses[i] - previous_poses[i - 1]);
  }
  velocity /= (n - 1);

  // Predict the next position by adding average velocity to the last position
  voxblox::Point predicted_position = previous_poses.back() + velocity;
  return predicted_position;
}

}  // namespace dynablox
