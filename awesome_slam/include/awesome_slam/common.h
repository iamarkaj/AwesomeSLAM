/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Arkajyoti Basak
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Arkajyoti Basak nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ASLAM_COMMON_H
#define ASLAM_COMMON_H

#include <awesome_slam_msgs/Landmarks.h>

#include <eigen3/Eigen/Core>

using Eigen::VectorXd;

/// \brief State transition function aka f
VectorXd stateTransitionFunction(const uint32_t N, const VectorXd &point,
                                 const float &vx, const float &az,
                                 const float &deltaTime) {
  VectorXd P;
  P = point.head(N);

  // Avoid division by 0
  if (std::fabs(az) > 0.001) {
    float r = vx / az;
    P(0) += r * (-std::sin(point(2)) + std::sin(point(2) + az * deltaTime));
    P(1) += r * (std::cos(point(2)) - std::cos(point(2) + az * deltaTime));
  } else {
    P(0) += vx * deltaTime * std::cos(point(2));
    P(1) += vx * deltaTime * std::sin(point(2));
  }

  P(2) += az * deltaTime;

  // Add noise only for augmented matrix
  if (point.size() > N) {
    P(0) += 0.5 * deltaTime * deltaTime * point(N) * std::cos(point(2));
    P(1) += 0.5 * deltaTime * deltaTime * point(N) * std::sin(point(2));
    P(2) += 0.5 * deltaTime * deltaTime * az;
  }

  return P;
}

/// \brief Measurement function aka h
VectorXd measurementFunction(const uint32_t N, const VectorXd &point) {
  VectorXd P;
  P = point;

  for (uint32_t i = 0; i < N - 3; i += 2) {
    P(3 + i) = std::sqrt(std::pow(point(3 + i) - point(0), 2) +
                         std::pow(point(4 + i) - point(1), 2));
    P(4 + i) =
        std::atan2(point(4 + i) - point(1), point(3 + i) - point(0)) - point(2);
  }

  return P;
}

/// \brief Convert X matrix to ROS msg
awesome_slam_msgs::Landmarks convertToLandmarkMsg(const uint32_t N,
                                                  const VectorXd &X) {
  std::vector<double> predX;
  std::vector<double> predY;

  for (uint32_t i = 0; i < N - 3; i += 2) {
    predX.push_back(X(3 + i));
    predY.push_back(X(4 + i));
  }

  awesome_slam_msgs::Landmarks L;
  L.x = predX;
  L.y = predY;

  return L;
}

#endif  // ASLAM_COMMON_H