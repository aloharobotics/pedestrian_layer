// PedestrianLayer.cpp --- 
// 
// Filename: PedestrianLayer.cpp
// Description: 
// Author: Federico Boniardi
// Maintainer: 
// Created: Fri Jun 20 15:32:53 2014 (+0100)
// Version: Wed Jun 25 13:45:52 2014 (+0100)
// Last-Updated: 
//           By: 
//     Update #: 4
// URL: 
// Keywords: 
// Compatibility: 
// 
// 

// Commentary: 
// 
// 
// 
// 

// Change Log:
// 
// 
// 
// 
// The MIT License (MIT)
// 
// Copyright (c) 2014 Federico Boniardi
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// 

// Code:


#include "pedestrian_layer/PedestrianLayer.hpp"
#include "target_to_pedestrian/TargetToPedestrian.h"
#include <cmath>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(pedestrian_layer::PedestrianLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

namespace pedestrian_layer {

double l2distance(std::pair<double,double> x1, std::pair<double,double> x2)
{
  return std::sqrt(x1.first * x1.first + x2.second * x2.second);
}

PedestrianLayer::PedestrianLayer() {}

void PedestrianLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  std::string pedestrians_topic;
  double costmap_update_freq, resolution, inflation_radius;
    
  current_ = true;

  if( !nh.getParam("pedestrians_topic", pedestrians_topic) )
    pedestrians_topic = "/agent_1/PTrackingBridge/targetEstimations";
  nh.param("/move_base/local_costmap/pedestrian_layer/update_steps", update_steps_, 1);
  nh.param("/move_base/local_costmap/pedestrian_layer/update_freq", update_freq_, 1.0);
  nh.param("/move_base/local_costmap/pedestrian_layer/decay_constant", decay_constant_, 10.0);
  nh.param("/move_base/local_costmap/pedestrian_layer/pedestrian_radius", pedestrian_inflation_radius_, 0.5);
  nh.param("/move_base/local_costmap/resolution", resolution, 0.05);
  nh.param("/move_base/local_costmap/inflation_radius", inflation_radius, 0.5);
  nh.param("/move_base/local_costmap/update_frequency", costmap_update_freq, 5.0);

  window_radius_ = int((0.5*inflation_radius+0.5*pedestrian_inflation_radius_)/resolution);
  
  update_costs_ = false;

  if( update_freq_ > costmap_update_freq ) {
    ROS_WARN("update rate of 'pedestrian_layer' is greater than 'update_frequency' of local_costmap. update_freq will be set to %f ", 0.2*costmap_update_freq);
    update_ratio_ = 5;
  } else {
    update_ratio_ = int(costmap_update_freq/update_freq_);
  }

  count_ = -1;
  
  sub_ = nh.subscribe(pedestrians_topic, 1000, &PedestrianLayer::getTargets, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &PedestrianLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void PedestrianLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void PedestrianLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, 
                                   double* min_x, double* min_y, double* max_x, double* max_y)
{
  if ( !enabled_ )
    return;

  if( ++count_ % update_ratio_ == 0 ) {
    count_ %= update_steps_;
    clearing_ = mark_;
    mark_.clear();

    for(unsigned int i=0; i<pedestrian_.size(); ++i) {
      double mark_x = pedestrian_[i].pose.x  + update_steps_ * pedestrian_[i].velocity * std::cos(pedestrian_[i].pose.theta) / update_freq_ ;
      double mark_y = pedestrian_[i].pose.y  + update_steps_ * pedestrian_[i].velocity * std::sin(pedestrian_[i].pose.theta) / update_freq_ ;
      std::pair<double, double> mark(mark_x, mark_y);
      if( l2distance(mark, std::pair<double, double>(robot_x, robot_y)) > 0.40 ) {
        mark_.push_back(mark);
        *min_x = std::min(*min_x, mark_x);
        *min_y = std::min(*min_y, mark_y);
        *max_x = std::max(*max_x, mark_x);
        *max_y = std::max(*max_y, mark_y);
      }
    }
    update_costs_ = true;
  }
}

void PedestrianLayer::updateCosts(costmap_2d::Costmap2D& master_grid, 
                                      int min_i, int min_j, int max_i, int max_j)
{
  if( !enabled_ )
      return;

  if( update_costs_ ) {
    unsigned int mx, cx;
    unsigned int my, cy;
    for(unsigned int i=0; i<clearing_.size(); ++i) {
      if( master_grid.worldToMap(clearing_[i].first, clearing_[i].second, cx, cy) ) { 
        for(int i=(-1)*window_radius_; i<window_radius_+1; ++i) 
          for(int j=(-1)*window_radius_; j<window_radius_+1; ++j) 
            master_grid.setCost(cx-i, cy-j, FREE_SPACE);
      }
    }
    for(unsigned int i=0; i<mark_.size(); ++i) {
      if( master_grid.worldToMap(mark_[i].first, mark_[i].second, mx, my) ) {
        for(int i=(-1)*window_radius_; i<window_radius_+1; ++i) 
          for(int j=(-1)*window_radius_; j<window_radius_+1; ++j) 
            master_grid.setCost(mx-i, my-j, this->pedestrian_cost(mx,my,i,j));
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
    }
    update_costs_ = false;
  }
}

void PedestrianLayer::getTargets(const PTrackingBridge::TargetEstimations::ConstPtr& tgts_msg)
{
  // Target estimations are in world frame. To convert to map frame swap x=y,y=-x
  pedestrian_.clear();
  for(unsigned int i=0; i<tgts_msg->identities.size(); ++i) {
    target_to_pedestrian::PedestrianEstimation ped;
    ped.id = tgts_msg->identities.at(i);
    ped.pose.x = tgts_msg->positions.at(i).y; 
    ped.pose.y = -tgts_msg->positions.at(i).x; 
    ped.velocity = std::sqrt(std::pow(tgts_msg->velocities.at(i).x, 2) +
                             std::pow(tgts_msg->velocities.at(i).y, 2));
    ped.pose.theta = (ped.velocity > 0 ? std::asin(tgts_msg->velocities.at(i).y/ped.velocity) : 0); // reverse the angle
    ped.std_deviation_x = std::pow(tgts_msg->standardDeviations.at(i).y, 2);
    ped.std_deviation_y = std::pow(tgts_msg->standardDeviations.at(i).x, 2);
    pedestrian_.push_back(ped);
  }
}

int  PedestrianLayer::pedestrian_cost(unsigned int mx, unsigned int my, int i, int j)
{
  double d = std::sqrt(double(i*i+j*j)); 
  return int(std::exp(-0.001*decay_constant_*d*d)*(INSCRIBED_INFLATED_OBSTACLE-1)*(d <= window_radius_)); 
}

}
// 
// PedestrianLayer.cpp ends here
