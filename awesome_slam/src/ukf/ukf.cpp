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
    subOdom = nh.subscribe("/odom", 1, &aslam::UKFSlam::cbOdom, this);
    subSensorLM = nh.subscribe("/out/landmarks/sensor", 1, &aslam::UKFSlam::cbSensorLandmark, this);
    pubLandmarks = nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks/kalman", 1);

    aslam::UKFSlam::initialize();
}

/// \brief Initialize
void aslam::UKFSlam::initialize()
{
    N = 3;
    initX = true;
    initZwithLaser = true;
    lastTime = ros::Time::now().toSec();

    param.X = VectorXd::Zero(N);
    param.Z = VectorXd::Zero(N);
    param.Q = MatrixXd::Zero(N, N);

    param.R = MatrixXd::Identity(N, N) * 0.2;
    param.P = MatrixXd::Identity(N, N) * 0.001;

    param.Q(0, 0) = 0.001;
    param.Q(1, 1) = 0.001;
    param.Q(2, 2) = 0.001;
    param.P.bottomRightCorner(N - 3, N - 3) = MatrixXd::Identity(N - 3, N - 3) * 1.0;
}

/// \brief Odom callback
void aslam::UKFSlam::cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (initZwithLaser)
        return;

    // Calculate delta time
    float deltaTime = std::min(ros::Time::now().toSec() - lastTime, 1.0);
    lastTime = ros::Time::now().toSec();

    // Update Z with new measurement data
    aslam::UKFSlam::updateZ(msg);

    // Initialize X with Z only once
    if (initX)
    {
        initX = false;
        param.X = param.Z;
    }

    // SLAM
    aslam::UKFSlam::slam(msg->twist.twist.linear.x, msg->twist.twist.angular.z, deltaTime);

    // Publish landmarks to visualize in Rviz
    awesome_slam_msgs::Landmarks L = convertToLandmarkMsg(N, param.X);
    pubLandmarks.publish(L);
}

/// \brief Update landmarks: range and bearing from laser data
void aslam::UKFSlam::cbSensorLandmark(const awesome_slam_msgs::Landmarks::ConstPtr &msg)
{
    initZwithLaser = false;
    sensorMeasuredLM.clear();
    uint32_t size = msg->x.size();

    LaserData data;
    for (uint32_t i = 0; i < size; ++i)
    {
        data.assign(msg->x[i], msg->y[i]);
        sensorMeasuredLM.push_back(data);
    }
}

/// \brief Update Z
void aslam::UKFSlam::updateZ(const nav_msgs::Odometry::ConstPtr &msg)
{
    float theta =
        std::atan2(2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                        msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
                   1 - 2 * (std::pow(msg->pose.pose.orientation.z, 2) + std::pow(msg->pose.pose.orientation.y, 2)));

    param.Z(0) = msg->pose.pose.position.x;
    param.Z(1) = msg->pose.pose.position.y;
    param.Z(2) = theta;

    // Landmark Data Association
    std::vector<LaserData> NewLandmarkList;

    for (LaserData &data : sensorMeasuredLM)
    {
        // Normalize
        data.bearing = normalizeAngle(data.bearing);

        // Initially send all landmark to waiting list
        if (N == 3)
        {
            aslam::UKFSlam::sendToNewLandmarkWaiting(data);
            continue;
        }

        // Find nearest neighbour
        uint32_t corrIdx = 0;
        Point oldLM(param.X(3), param.X(4));
        Point newLM = data.toPoint(param.Z);
        float mindist = newLM.distance(oldLM);
        for (uint32_t j = 2; j < N - 3; j += 2)
        {
            oldLM.assign(param.X(3 + j), param.X(4 + j));
            float dist = newLM.distance(oldLM);
            if (dist < mindist)
            {
                corrIdx = j;
                mindist = dist;
            }
        }

        if (mindist < MIN_DIST_THRESH)
        {
            // Associate with the correct index
            param.Z(3 + corrIdx) = data.range;
            param.Z(4 + corrIdx) = data.bearing;
            continue;
        }
        else
        {
            // If landmark cannot be associated with any current landmark
            aslam::UKFSlam::sendToNewLandmarkWaiting(data);
        }
    }

    // Push landmarks from NewLandmarkWaitingList to NewLandmarkList
    for (auto &waitingData : NewLandmarkWaitingList)
    {
        if (waitingData.second >= MIN_LANDMARK_OCC)
        {
            NewLandmarkList.push_back(waitingData.first);
            waitingData.second = -1;
        }
    }

    if (NewLandmarkList.size())
    {
        aslam::UKFSlam::addNewLandmark(NewLandmarkList);
        NewLandmarkList.clear();
    }
}

/// \brief If point is already present in NewLandmarkWaitingList increment
///        its count else push as new landmark in NewLandmarkWaitingList
void aslam::UKFSlam::sendToNewLandmarkWaiting(const LaserData &data)
{
    // Push if NewLandmarkWaitingList is emtpy
    if (NewLandmarkWaitingList.empty())
    {
        NewLandmarkWaitingList.push_back({data, 1});
        return;
    }

    // Find nearest neighbour
    uint32_t corrIdx = 0;
    Point newLM = data.toPoint(param.Z);
    Point oldLM = NewLandmarkWaitingList[0].first.toPoint(param.Z);
    float mindist = newLM.distance(oldLM);
    uint32_t size = NewLandmarkWaitingList.size();
    for (uint32_t i = 1; i < size; ++i)
    {
        oldLM.assign(NewLandmarkWaitingList[i].first.toPoint(param.Z));
        float dist = newLM.distance(oldLM);
        if (dist < mindist)
        {
            corrIdx = i;
            mindist = dist;
        }
    }

    if (mindist < MIN_DIST_THRESH)
    {
        // Increment its count
        NewLandmarkWaitingList[corrIdx].second++;
    }
    else
    {
        // Push as new landmark
        NewLandmarkWaitingList.push_back({data, 1});
    }
}

void aslam::UKFSlam::addNewLandmark(const std::vector<LaserData> &NewLandmarkList)
{
    uint32_t cacheN = N;
    uint32_t size = NewLandmarkList.size();

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
    param.P.conservativeResizeLike(MatrixXd::Identity(N, N) * 1.0);
    param.R.conservativeResizeLike(MatrixXd::Identity(N, N) * 0.2);

    // Update weight matrix
    param.updateWeights(N);

    for (uint32_t i = 0; i < size * 2; i += 2)
    {
        // Update Z
        param.Z(cacheN + i) = NewLandmarkList[i / 2].range;
        param.Z(cacheN + i + 1) = NewLandmarkList[i / 2].bearing;

        // Update X
        param.X(cacheN + i) = param.Z(0) + param.Z(cacheN + i) * std::cos(param.Z(2) + param.Z(cacheN + i + 1));
        param.X(cacheN + i + 1) = param.Z(1) + param.Z(cacheN + i) * std::sin(param.Z(2) + param.Z(cacheN + i + 1));
    }
}

/// \brief SLAM
void aslam::UKFSlam::slam(const float &vx, const float &az, const float &deltaTime)
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
    Paug(N, N) = param.std_a * param.std_a;
    Paug(N + 1, N + 1) = param.std_yaw * param.std_yaw;

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
        XsigPred.col(i) = stateTransitionFunction(N, XsigAug.col(i), vx, az, deltaTime);
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
    ros::Rate rate(1);
    aslam::UKFSlam a;
    std::cerr << "[UKF] Node started!\n";

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}