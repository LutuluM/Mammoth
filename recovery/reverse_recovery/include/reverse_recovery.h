#ifndef reverse_recovery_
#define reverse_recovery_
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <string>

namespace reverse_recovery{
  class ReverseRecovery : public nav_core::RecoveryBehavior {
    public:
      ReverseRecovery();
      void initialize(std::string name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);
      void runBehavior();
      ~ReverseRecovery();

    private:
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      costmap_2d::Costmap2D costmap_;
      std::string name_;
      tf::TransformListener* tf_;
      bool initialized_;
      base_local_planner::CostmapModel* world_model_;
  };
};
#endif