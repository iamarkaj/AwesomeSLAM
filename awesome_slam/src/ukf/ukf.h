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

#ifndef ASLAM_UKF_SLAM_H
#define ASLAM_UKF_SLAM_H

#include <awesome_slam/common.h>
#include <awesome_slam/config.h>
#include <awesome_slam/structures.h>
#include <awesome_slam/tools.h>
#include <awesome_slam_msgs/Landmarks.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct Parameters
{
    VectorXd X;
    VectorXd Z;
    VectorXd weights;

    MatrixXd P;
    MatrixXd Q;
    MatrixXd R;

    float std_a;
    float std_yaw;
    float lambda;

    Parameters()
    {
        std_a = 0.2;
        std_yaw = 0.2;
        updateWeights(3);
    }

    void updateWeights(int N)
    {
        lambda = 3.0 - (N + 2);

        // Set weight vector
        float weight = 0.5 / (lambda + N + 2);
        weights = VectorXd::Ones(2 * N + 5) * weight;
        weights(0) = lambda / (lambda + N + 2);
    }
};

namespace aslam
{
class UKFSlam
{
  public:
    UKFSlam();

  private:
    ros::NodeHandle nh;
    ros::Subscriber subOdom;
    ros::Subscriber subSensorLM;
    ros::Publisher pubLandmarks;

    uint32_t N;
    bool initX;
    float lastTime;
    bool initZwithLaser;

    std::vector<LaserData> sensorMeasuredLM;
    std::vector<std::pair<LaserData, uint32_t>> NewLandmarkWaitingList;

    Parameters param;

    void initialize();
    void cbOdom(const nav_msgs::Odometry::ConstPtr &msg);
    void cbSensorLandmark(const awesome_slam_msgs::Landmarks::ConstPtr &msg);

    void updateZ(const nav_msgs::Odometry::ConstPtr &msg);
    void sendToNewLandmarkWaiting(const LaserData &data);
    void addNewLandmark(const std::vector<LaserData> &NewLandmarkList);
    void slam(const float &vx, const float &az, const float &deltaTime);
};
} // namespace aslam

#endif // ASLAM_UKF_SLAM_H