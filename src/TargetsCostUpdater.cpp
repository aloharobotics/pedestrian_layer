// TargetsCostUpdater.cpp --- 
// 
// Filename: TargetsCostsUpdater.cpp
// Description: 
// Author: Federico Boniardi
// Maintainer: 
// Created: Fri Jun 20 19:54:47 2014 (+0100)
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

#include "pedestrian_layer/TargetsCostUpdater.hpp"

using geometry_msgs::Point32;

namespace pedestrian_layer {

TargetsCostUpdater::TargetsCostUpdater() {}

TargetsCostUpdater::~TargetsCostUpdater() {}

void TargetsCostUpdater::init(ros::NodeHandle& nh)
{
  nh.param("update_freq", update_freq, 10.0);
  nh.param("time_steps", time_steps, 1);
  counter = 0;
}

void TargetsCostUpdater::drawPedestrians(Point32 positions[], Point32 velocities[],
                                         Point32 eig_val_1[], Point32 eig_val_2[])
{


}

}

// 
// TargetsCostsUpdater.cpp ends here
