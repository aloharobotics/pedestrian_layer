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
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &PedestrianLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  tcu.init(nh, name_);
}

void PedestrianLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void PedestrianLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, 
                                   double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  mark_x_ = origin_x + cos(origin_yaw);
  mark_y_ = origin_y + sin(origin_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void PedestrianLayer::updateCosts(costmap_2d::Costmap2D& master_grid, 
                                  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}

} 

// 
// PedestrianLayer.cpp ends here
