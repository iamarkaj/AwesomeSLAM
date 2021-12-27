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

#ifndef ASLAM_TOOLS_H
#define ASLAM_TOOLS_H

#include <awesome_slam/config.h>

#include <eigen3/Eigen/Core>

/// \brief Normalize angle
inline float normalizeAngle(const float &theta)
{
        float ret = std::fmod(theta, 2 * PI);
        ret = ret > PI ? ret - 2 * PI : ret;
        ret = ret < -PI ? ret + 2 * PI : ret;
        return ret;
}

/// \brief Calculate the distance between two points
inline float eulerDistance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
{
        float dx = p1(0) - p2(0);
        float dy = p1(1) - p2(1);
        float ret = std::sqrt(dx * dx + dy * dy);
        return ret;
}

/// \brief Convert Quaternion angle to Euler angle
inline float quat2euler(const float &w, const float &x, const float &y, const float &z)
{
        float yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y));
        return yaw;
}

#endif // ASLAM_TOOLS_H