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

#ifndef ASLAM_CONFIG_H
#define ASLAM_CONFIG_H

const float PI = 3.141592654;
const float DEG2RAD = 0.01745329251;

const int FREQ = 1;
const float MIN_DIST_THRESH = 0.5;
const int MIN_LANDMARK_OCC = 10;
const int MAX_LANDMARK_COUNT = 30;

// Sensor Landmark
const float STD = 0.4;
const float MIN_MEAN = 1.5;
const float MAX_MEAN = 3.0;
const int MIN_CLUSTER_POINTS = 3;

// UKF
const float UKF_STD_A = 0.2;
const float UKF_STD_YAW = 0.2;
const float UKF_KP_ROBOT_POSE = 0.001;
const float UKF_KP_LANDMARK_POSE = 1.0;
const float UKF_KR = 0.2;
const float UKF_KQ = 0.001;

// EKF
const float EKF_KP_ROBOT_POSE = 0.001;
const float EKF_KP_LANDMARK_POSE = 10000.0;
const float EKF_KR = 0.2;
const float EKF_KQ = 0.001;


#endif // ASLAM_CONFIG_H