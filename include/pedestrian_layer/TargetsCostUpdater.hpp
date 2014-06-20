// TargetsCostUpdater.hpp --- 
// 
// Filename: TargetsCostUpdater.hpp
// Description: 
// Author: Federico Boniardi
// Maintainer: Federico Boniardi
// Created: Fri Jun 20 15:31:32 2014 (+0100)
// Version: 0.1.0-SNAPSHOT
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

#ifndef TARGETSCOSTUPDATER_H
#define TARGETSCOSTUPDATER_H

#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Point32.h>
#include "PTrackingBridge/TargetEstimations.h"

using geometry_msgs::Point32;

namespace pedestrian_layer {

class TargetsCostUpdater
{
 public:
  TargetsCostUpdater();
  virtual ~TargetsCostUpdater();
  void init(ros::NodeHandle&, std::string);
  void drawPedestrians(Point32[], Point32[], Point32[], Point32[]);
 private:
  double update_freq;
  int time_steps;
  int counter;
};

}

#endif /* TARGETSCOSTUPDATER_H */

// 
// TargetsCostUpdater.hpp ends here
