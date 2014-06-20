// PedestrianLayer.cpp --- 
// 
// Filename: PedestrianLayer.cpp
// Description: 
// Author: Federico Boniardi
// Maintainer: 
// Created: Fri Jun 20 15:32:53 2014 (+0100)
// Version: 
// Last-Updated: 
//           By: 
//     Update #: 0
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
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(pedestrian_layer::PedestrianLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace pedestrian_layer {

PedestrianLayer::PedestrianLayer() {}

void PedestrianLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = costmap_2d::NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &PedestrianLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
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

void PedestrianLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, 
                                       double* min_x, double* min_y, double* max_x, double* max_y)
{
  if ( !enabled_ )
    return;

  double mark_x = origin_x + cos(origin_yaw), mark_y = origin_y + sin(origin_yaw);
  unsigned int mx;
  unsigned int my;
  if( worldToMap(mark_x, mark_y, mx, my) ) {
    setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
  }
  
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
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

bool isDiscretized() 
{
  return true;
}

} 

// 
// PedestrianLayer.cpp ends here
