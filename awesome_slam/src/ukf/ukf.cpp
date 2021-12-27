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

#include "ukf.h"

/// \brief Constructor
aslam::UKFSlam::UKFSlam() : nh(ros::NodeHandle())
{
        sub_odom = nh.subscribe("/odom", 10, &aslam::UKFSlam::cbOdom, this);
        sub_sensor_landmark = nh.subscribe("/out/landmarks/sensor", 10, &aslam::UKFSlam::cbSensorLandmark, this);
        pub_landmark = nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks/kalman", 1);

        aslam::UKFSlam::initialize();
}

/// \brief Initialize
void aslam::UKFSlam::initialize()
{
        N = 3;
        init_x = true;
        init_z = true;
        last_time = ros::Time::now().toSec();

        param.X = VectorXd::Zero(N);
        param.Z = VectorXd::Zero(N);
        param.Q = MatrixXd::Zero(N, N);

        param.R = MatrixXd::Identity(N, N) * UKF_KR;
        param.P = MatrixXd::Identity(N, N) * UKF_KP_ROBOT_POSE;

        param.Q(0, 0) = UKF_KQ;
        param.Q(1, 1) = UKF_KQ;
        param.Q(2, 2) = UKF_KQ;
        param.P.bottomRightCorner(N - 3, N - 3) = MatrixXd::Identity(N - 3, N - 3) * UKF_KP_LANDMARK_POSE;
}

/// \brief Odom callback
void aslam::UKFSlam::cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
        if (init_z)
                return;

        // Calculate delta time
        float delta_time = std::min(ros::Time::now().toSec() - last_time, 1.0);
        last_time = ros::Time::now().toSec();

        // Update Z with new measurement data
        aslam::UKFSlam::updateZ(msg);

        // Initialize X with Z only once
        if (init_x)
        {
                init_x = false;
                param.X = param.Z;
        }

        // SLAM
        aslam::UKFSlam::slam(msg->twist.twist.linear.x, msg->twist.twist.angular.z, delta_time);

        // Publish landmarks to visualize in Rviz
        awesome_slam_msgs::Landmarks L = convertToLandmarkMsg(N, param.X);
        pub_landmark.publish(L);
}

/// \brief Update landmarks: range and bearing from laser data
void aslam::UKFSlam::cbSensorLandmark(const awesome_slam_msgs::Landmarks::ConstPtr &msg)
{
        init_z = false;
        sensor_landmark.clear();
        uint32_t size = msg->x.size();

        LaserData data;
        for (uint32_t i = 0; i < size; ++i)
        {
                data.assign(msg->x[i], msg->y[i]);
                sensor_landmark.push_back(data);
        }
}

/// \brief Update Z
void aslam::UKFSlam::updateZ(const nav_msgs::Odometry::ConstPtr &msg)
{
        param.Z(0) = msg->pose.pose.position.x;
        param.Z(1) = msg->pose.pose.position.y;
        param.Z(2) = quat2euler(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z);

        // Landmark Data Association
        std::vector<LaserData> new_landmark;

        for (LaserData &data : sensor_landmark)
        {
                // Normalize
                data.bearing = normalizeAngle(data.bearing);

                // Initially send all landmark to waiting list
                if (N == 3)
                {
                        aslam::UKFSlam::updateNewLandmarkWait(data);
                        continue;
                }

                // Find nearest neighbour
                uint32_t corr_id = 0;
                Point landmark_1(param.X(3), param.X(4));
                Point landmark_2 = data.toPoint(param.Z);
                float mindist = landmark_2.distance(landmark_1);
                for (uint32_t j = 2; j < N - 3; j += 2)
                {
                        landmark_1.assign(param.X(3 + j), param.X(4 + j));
                        float dist = landmark_2.distance(landmark_1);
                        if (dist < mindist)
                        {
                                corr_id = j;
                                mindist = dist;
                        }
                }

                if (mindist < MIN_DIST_THRESH)
                {
                        // Associate with the correct index
                        param.Z(3 + corr_id) = data.range;
                        param.Z(4 + corr_id) = data.bearing;
                        continue;
                }
                else
                {
                        // If landmark cannot be associated with any current landmark
                        aslam::UKFSlam::updateNewLandmarkWait(data);
                }
        }

        // Push landmarks from new_landmark_wait to new_landmark
        for (auto &waitingData : new_landmark_wait)
        {
                if (waitingData.second == MIN_LANDMARK_OCC)
                {
                        new_landmark.push_back(waitingData.first);
                        waitingData.second += 1;
                }
        }

        if (new_landmark.size())
        {
                aslam::UKFSlam::updateNewLandmark(new_landmark);
                new_landmark.clear();
        }
}

/// \brief If point is already present in new_landmark_wait increment
///        its count else push as new landmark in new_landmark_wait
void aslam::UKFSlam::updateNewLandmarkWait(const LaserData &data)
{
        // Push if new_landmark_wait is emtpy
        if (new_landmark_wait.empty())
        {
                new_landmark_wait.push_back({data, 1});
                return;
        }

        // Find nearest neighbour
        uint32_t corr_id = 0;
        Point landmark_2 = data.toPoint(param.Z);
        Point landmark_1 = new_landmark_wait[0].first.toPoint(param.Z);
        float mindist = landmark_2.distance(landmark_1);
        uint32_t size = new_landmark_wait.size();
        for (uint32_t i = 1; i < size; ++i)
        {
                landmark_1.assign(new_landmark_wait[i].first.toPoint(param.Z));
                float dist = landmark_2.distance(landmark_1);
                if (dist < mindist)
                {
                        corr_id = i;
                        mindist = dist;
                }
        }

        if (mindist < MIN_DIST_THRESH)
        {
                // Increment its count
                new_landmark_wait[corr_id].second++;
        }
        else
        {
                // Push as new landmark
                new_landmark_wait.push_back({data, 1});
        }
}

void aslam::UKFSlam::updateNewLandmark(const std::vector<LaserData> &new_landmark)
{
        uint32_t cacheN = N;
        uint32_t size = new_landmark.size();

        // New N
        N += 2 * size;

        if (N >= MAX_LANDMARK_COUNT)
        {
                std::cerr << "[WARN] MAXIMUM LANDMARK COUNT IS SET TO " << MAX_LANDMARK_COUNT << "\n";
                N = cacheN;
                return;
        }

        // Resize all matrices
        param.X.conservativeResizeLike(VectorXd::Zero(N));
        param.Z.conservativeResizeLike(VectorXd::Zero(N));
        param.Q.conservativeResizeLike(MatrixXd::Zero(N, N));
        param.P.conservativeResizeLike(MatrixXd::Identity(N, N) * UKF_KP_LANDMARK_POSE);
        param.R.conservativeResizeLike(MatrixXd::Identity(N, N) * UKF_KR);

        // Update weight matrix
        param.updateWeights(N);

        for (uint32_t i = 0; i < size * 2; i += 2)
        {
                // Update Z
                param.Z(cacheN + i) = new_landmark[i / 2].range;
                param.Z(cacheN + i + 1) = new_landmark[i / 2].bearing;

                // Update X
                param.X(cacheN + i) = param.Z(0) + param.Z(cacheN + i) * std::cos(param.Z(2) + param.Z(cacheN + i + 1));
                param.X(cacheN + i + 1) = param.Z(1) + param.Z(cacheN + i) * std::sin(param.Z(2) + param.Z(cacheN + i + 1));
        }
}

/// \brief SLAM
void aslam::UKFSlam::slam(const float &vx, const float &az, const float &delta_time)
{
        VectorXd Xaug = VectorXd(N + 2);
        MatrixXd Paug = MatrixXd(N + 2, N + 2);

        // Create sigma point matrix
        MatrixXd XsigAug = MatrixXd(N + 2, 2 * N + 5);

        // Create augmented mean state
        Xaug.head(N) = param.X;
        Xaug(N) = 0.0;
        Xaug(N + 1) = 0.0;

        // Create augmented covariance matrix
        Paug.fill(0.0);
        Paug.topLeftCorner(N, N) = param.P;
        Paug(N, N) = UKF_STD_A * UKF_STD_A;
        Paug(N + 1, N + 1) = UKF_STD_YAW * UKF_STD_YAW;

        // Create square root matrix
        MatrixXd L = Paug.llt().matrixL();

        // Create augmented sigma points
        XsigAug.col(0) = Xaug;
        float w = std::sqrt(param.lambda + N + 2);
        for (uint32_t i = 0; i < N + 2; ++i)
        {
                XsigAug.col(i + 1) = Xaug + w * L.col(i);
                XsigAug.col(i + 3 + N) = Xaug - w * L.col(i);
        }

        // Create matrix with predicted sigma points as columns
        MatrixXd XsigPred = MatrixXd(N, 2 * N + 5);
        for (uint32_t i = 0; i < 2 * N + 5; ++i)
        {
                XsigPred.col(i) = stateTransitionFunction(N, XsigAug.col(i), vx, az, delta_time);
                XsigPred.col(i)(2) = normalizeAngle(XsigPred.col(i)(2));
        }

        // Predicted state mean
        param.X.fill(0.0);
        for (uint32_t i = 0; i < 2 * N + 5; ++i)
        {
                param.X += param.weights(i) * XsigPred.col(i);
        }

        // Predicted state covariance matrix
        VectorXd Xdiff;
        param.P.fill(0.0);
        for (uint32_t i = 0; i < 2 * N + 5; ++i)
        {
                // State difference
                Xdiff = XsigPred.col(i) - param.X;
                Xdiff(2) = normalizeAngle(Xdiff(2));

                param.P += param.weights(i) * Xdiff * Xdiff.transpose();
        }

        // Add noise
        param.P = param.P + param.Q;

        // Transform sigma points into measurement space
        MatrixXd Zsig = MatrixXd(N, 2 * N + 5);
        for (uint32_t i = 0; i < 2 * N + 5; ++i)
        {
                Zsig.col(i) = measurementFunction(N, XsigPred.col(i));
        }

        // Mean predicted measurement
        VectorXd Zpred = VectorXd::Zero(N);
        for (uint32_t i = 0; i < 2 * N + 5; ++i)
        {
                Zpred += param.weights(i) * Zsig.col(i);
        }

        // Normalize
        for (uint32_t j = 0; j < N - 1; j += 2)
        {
                Zpred(2 + j) = normalizeAngle(Zpred(2 + j));
        }

        // Innovation covariance matrix S
        VectorXd Zdiff;
        MatrixXd S = MatrixXd::Zero(N, N);
        for (uint32_t i = 0; i < 2 * N + 5; ++i)
        {
                // Residual
                Zdiff = Zsig.col(i) - Zpred;
                for (uint32_t j = 0; j < N - 1; j += 2)
                {
                        Zdiff(2 + j) = normalizeAngle(Zdiff(2 + j));
                }

                S = S + param.weights(i) * Zdiff * Zdiff.transpose();
        }

        // Add measurement noise covariance matrix
        S = S + param.R;

        // Calculate cross correlation matrix
        MatrixXd Tc = MatrixXd::Zero(N, N);
        for (uint32_t i = 0; i < 2 * N + 5; ++i)
        {
                // Residual
                Zdiff = Zsig.col(i) - Zpred;
                for (uint32_t j = 0; j < N - 1; j += 2)
                {
                        Zdiff(2 + j) = normalizeAngle(Zdiff(2 + j));
                }

                // State difference
                Xdiff = XsigPred.col(i) - param.X;
                Xdiff(2) = normalizeAngle(Xdiff(2));

                Tc = Tc + param.weights(i) * Xdiff * Zdiff.transpose();
        }

        // Kalman gain K
        MatrixXd K = Tc * S.inverse();

        // Residual
        Zdiff = param.Z - Zpred;

        for (uint32_t j = 0; j < N - 1; j += 2)
        {
                Zdiff(2 + j) = normalizeAngle(Zdiff(2 + j));
        }

        // Update state mean and covariance matrix
        param.X = param.X + K * Zdiff;

        param.P = param.P - K * S * K.transpose();
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "aslam_ukf");
        ros::Time::init();
        ros::Rate rate(FREQ);
        aslam::UKFSlam a;
        std::cerr << "[UKF] Node started!\n";

        while (ros::ok())
        {
                ros::spinOnce();
                rate.sleep();
        }
}