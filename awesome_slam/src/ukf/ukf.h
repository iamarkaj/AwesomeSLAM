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

        float lambda;

        Parameters()
        {
                updateWeights(3);
        }

        void updateWeights(uint32_t N)
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

      private:
        ros::Subscriber sub_odom;

      private:
        ros::Subscriber sub_sensor_landmark;

      private:
        ros::Publisher pub_landmark;

      private:
        uint32_t N;

      private:
        bool init_x;

      private:
        bool init_z;

      private:
        float last_time;

      private:
        std::vector<LaserData> sensor_landmark;

      private:
        std::vector<std::pair<LaserData, uint32_t>> new_landmark_wait;

      private:
        Parameters param;

      private:
        void initialize();

      private:
        void cbOdom(const nav_msgs::Odometry::ConstPtr &msg);

      private:
        void cbSensorLandmark(const awesome_slam_msgs::Landmarks::ConstPtr &msg);

      private:
        void updateZ(const nav_msgs::Odometry::ConstPtr &msg);

      private:
        void updateNewLandmarkWait(const LaserData &data);

      private:
        void updateNewLandmark(const std::vector<LaserData> &new_landmark);

      private:
        void slam(const float &vx, const float &az, const float &delta_time);
};
} // namespace aslam

#endif // ASLAM_UKF_SLAM_H