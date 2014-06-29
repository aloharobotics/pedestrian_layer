// PedestrianLayer.hpp --- 
// 
// Filename: PedestrianLayer.hpp
// Description: 
// Author: Federico Boniardi
// Maintainer: 
// Created: Fri Jun 20 15:33:40 2014 (+0100)
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

#ifndef PEDESTRIANLAYER_H
#define PEDESTRIANLAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include "target_to_pedestrian/PedestrianEstimation.h"
#include "PTrackingBridge/TargetEstimations.h"

namespace pedestrian_layer {

class PedestrianLayer : public costmap_2d::Layer
{
 public:
  PedestrianLayer();
  virtual void onInitialize();
  virtual void updateBounds(double, double, double, double*, double*, double*, double*);
  virtual void updateCosts(costmap_2d::Costmap2D&, int, int, int, int);
 private:
  void reconfigureCB(costmap_2d::GenericPluginConfig&, uint32_t);
  void getTargets(const PTrackingBridge::TargetEstimations::ConstPtr&);
  int pedestrian_cost(unsigned int, unsigned int, int, int);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;  
  ros::Subscriber sub_;
  std::vector<target_to_pedestrian::PedestrianEstimation> pedestrian_;
  std::vector< std::pair<double,double> > clearing_;
  std::vector< std::pair<double,double> > mark_;
  int count_;
  int update_steps_, update_ratio_;
  double update_freq_;
  bool update_costs_;
  double pedestrian_inflation_radius_;
  int window_radius_;
  double decay_constant_;
};

}

#endif /* PEDESTRIANLAYER_H */

// 
// PedestrianLayer.hpp ends here
