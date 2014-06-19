#ifndef PEDESTRIANLAYER_H
#define PEDESTRIANLAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace pedestrian_layer {

class PedestrianLayer : public costmap_2d::Layer
{
 public:
  PedestrianLayer();
  ~PedestrianLayer();
  virtual void onInitialize();
  virtual void updateBounds(double, double, double, double*, double*, double*, double*);
  virtual void updateCosts(costmap_2d::Costmap2D&, int, int, int, int);
 private:
  void reconfigureCB(costmap_2d::GenericPluginConfig&, uint32_t);
  double mark_x_, mark_y_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;  
};

}

#endif /* PEDESTRIANLAYER_H */
