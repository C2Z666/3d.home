#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include "nav2_costmap_2d/voxel_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace d1_temporal_costmap
{
class TemporalVoxelLayer : public nav2_costmap_2d::VoxelLayer
{
public:
  void onInitialize() override
  {
    VoxelLayer::onInitialize();
    auto node = node_.lock();
    double memory = 0.0, rate = 0.0;
    node->get_parameter(name_ + ".obstacles.observation_persistence", memory);
    node->get_parameter(name_ + ".obstacles.expected_update_rate", rate);
    if (marking_buffers_.size() != 1 || memory < 0.5 || memory > 30.0 ||
      rate <= 0.0 || rate > 0.5)
    {
      throw std::runtime_error("TemporalVoxelLayer requires one obstacles source, bounded memory 0.5..30s and freshness <=0.5s");
    }
    RCLCPP_INFO(logger_, "Live obstacles expire after %.2fs without observations; static layer is independent", memory);
  }

  void updateBounds(double x, double y, double yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
    if (!enabled_) {return;}
    std::vector<nav2_costmap_2d::Observation> observations;
    const bool received = getMarkingObservations(observations);
    const double now = clock_->now().seconds();
    double latest = -std::numeric_limits<double>::infinity();
    for (const auto & observation : observations) {
      latest = std::max(latest, rclcpp::Time(observation.cloud_->header.stamp).seconds());
    }
    // A re-received old cloud is not fresh evidence. During sensor/TF loss,
    // preserve the previous layer and make Nav2 wait instead of clearing it.
    if (!received || now - latest > 0.5 || latest - now > 0.05) {
      current_ = false;
      recovery_since_ = -1.0;
      return;
    }
    if (recovery_since_ < 0.0) {recovery_since_ = now;}
    if (now - recovery_since_ < 0.3) {current_ = false; return;}

    // ObservationBuffer's persistence limits cloud history, not grid lifetime.
    // Rebuild THIS layer on each update so expired observations also leave its
    // cells. Do not call reset(): it refreshes sensor timestamps. The master
    // map and static layer are never reset here.
    resetMaps();
    double lx = std::numeric_limits<double>::max(), ly = lx;
    double ux = -lx, uy = -lx;
    VoxelLayer::updateBounds(x, y, yaw, &lx, &ly, &ux, &uy);
    // Include the previous occupied area, so vanished cells and their inflation
    // are recomposed from the static map even if no new point touches them.
    *min_x = std::min(*min_x, std::min(lx, previous_min_x_));
    *min_y = std::min(*min_y, std::min(ly, previous_min_y_));
    *max_x = std::max(*max_x, std::max(ux, previous_max_x_));
    *max_y = std::max(*max_y, std::max(uy, previous_max_y_));
    previous_min_x_ = lx; previous_min_y_ = ly;
    previous_max_x_ = ux; previous_max_y_ = uy;
  }
private:
  double recovery_since_ = -1.0;
  double previous_min_x_ = std::numeric_limits<double>::max();
  double previous_min_y_ = std::numeric_limits<double>::max();
  double previous_max_x_ = -std::numeric_limits<double>::max();
  double previous_max_y_ = -std::numeric_limits<double>::max();
};
}
PLUGINLIB_EXPORT_CLASS(d1_temporal_costmap::TemporalVoxelLayer, nav2_costmap_2d::Layer)
