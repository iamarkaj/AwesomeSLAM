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

#ifndef ASLAM_STRUCTURES_H
#define ASLAM_STRUCTURES_H

#include <awesome_slam/config.h>
#include <awesome_slam/tools.h>

#include <eigen3/Eigen/Core>

struct Point {
  Eigen::Vector2d point;

  Point() {}

  Point(const float &a, const float &b) : point(a, b) {}

  void assign(const float &a, const float &b) { point << a, b; }

  void assign(const Point &P) { point << P.point(0), P.point(1); }

  float operator()(const uint8_t i) const { return point(i); }

  float distance(const Point &P) const {
    float ret = EulerDistance(point, P.point);
    return ret;
  }

  bool operator==(const Point &P) const {
    float dist = EulerDistance(point, P.point);
    bool ret = dist < MIN_DIST_THRESH;
    return ret;
  }
};

struct LaserData {
  float range;
  float bearing;

  LaserData() {}

  LaserData(const float &a, const float &b) : range(a), bearing(b) {}

  void assign(const float &a, const float &b) {
    range = a;
    bearing = b;
  }

  Point toPoint(const Eigen::VectorXd &Z) const {
    float x = Z(0) + range * std::cos(Z(2) + bearing);
    float y = Z(1) + range * std::sin(Z(2) + bearing);
    Point ret(x, y);

    return ret;
  }
};

struct Circle {
  Eigen::Vector2d center;

  Circle() {}

  Circle(const float &a, const float &b) : center(a, b) {}

  LaserData toLaserData() const {
    float r = std::sqrt(center(0) * center(0) + center(1) * center(1));
    float b = std::atan2(center(1), center(0));
    LaserData ret(r, b);

    return ret;
  }
};

#endif  // ASLAM_STRUCTURES_H