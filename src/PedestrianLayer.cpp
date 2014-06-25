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

namespace pedestrian_layer {

double l2distance(std::pair<double,double> x1, std::pair<double,double> x2)
{
  return std::sqrt(x1.first * x1.first + x2.second * x2.second);
}

PedestrianLayer::PedestrianLayer() {}

void PedestrianLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = costmap_2d::NO_INFORMATION;
  matchSize();
  nh.param("update_steps", update_steps, 10);
  nh.param("update_freq", update_freq, 10.0);
  sub_ = nh.subscribe("ptracking_bridge", 1000, &PedestrianLayer::getTargets, this);
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &PedestrianLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  // FIX THESE LINES (add parameters)
  count = -1;
}

void PedestrianLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
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

  for(unsigned int i=0; i<clearing.size(); i++) {
    setCost(clearing[i].first, clearing[i].second, FREE_SPACE);
  }

  for (unsigned int i=0; i<pedestrian.size(); ++i) {    
    if( ++count % update_steps == 0 ) {
      // Drawing the pedestrians
      clearing.clear();
      double mark_x = pedestrian[i].pose.x + update_steps*pedestrian[i].velocity * std::cos(pedestrian[i].pose.theta) / update_freq;
      double mark_y = pedestrian[i].pose.y + update_steps*pedestrian[i].velocity * std::sin(pedestrian[i].pose.theta) / update_freq;
      unsigned int mx;
      unsigned int my;
      if( worldToMap(mark_x, mark_y, mx, my) ) {
        if( l2distance(std::pair<double,double>(mark_x, mark_y), std::pair<double,double>(robot_x, robot_y)) > 0.4 ) { // The robot itself is tracked by the kinect too
          setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
          clearing.push_back(std::pair<unsigned int, unsigned int>(mx, my));
        }
      } else {
        ROS_WARN("failed to update the map");
      }  
      *min_x = std::min(*min_x, mark_x);
      *min_y = std::min(*min_y, mark_y);
      *max_x = std::max(*max_x, mark_x);
      *max_y = std::max(*max_y, mark_y);
    } 
  }
}

void PedestrianLayer::updateCosts(costmap_2d::Costmap2D& master_grid, 
                                      int min_i, int min_j, int max_i, int max_j)
{
  if ( !enabled_ )
    return;
  
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = getIndex(i, j);
      if ( costmap_[index] == costmap_2d::NO_INFORMATION )
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
}

bool PedestrianLayer::isDiscretized() 
{
  return true;
}

void PedestrianLayer::getTargets(const PTrackingBridge::TargetEstimations::ConstPtr& tgts_msg)
{
  // Target estimations are already in map frame
  pedestrian.clear();
  for(unsigned int i=0; i<tgts_msg->identities.size(); ++i) {
    target_to_pedestrian::PedestrianEstimation ped;
    ped.id = tgts_msg->identities.at(i);
    ped.pose.x = tgts_msg->positions.at(i).x;
    ped.pose.y = tgts_msg->positions.at(i).y;
    ped.velocity = std::sqrt(std::pow(tgts_msg->velocities.at(i).x, 2) +
                             std::pow(tgts_msg->velocities.at(i).y, 2));
    ped.pose.theta = std::asin(tgts_msg->velocities.at(i).y/ped.velocity);
    ped.cov_position.data[0] = std::pow(tgts_msg->standardDeviations.at(i).x, 2);
    ped.cov_position.data[1] = std::pow(tgts_msg->standardDeviations.at(i).y, 2);
    pedestrian.push_back(ped);
  }
}

}
// 
// PedestrianLayer.cpp ends here
