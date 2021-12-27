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

#include "ekf.h"

/// \brief Constructor
aslam::EKFSlam::EKFSlam() : nh(ros::NodeHandle())
{
        sub_odom = nh.subscribe("/odom", 1, &aslam::EKFSlam::cbOdom, this);
        sub_sensor_landmark = nh.subscribe("/out/landmarks/sensor", 1, &aslam::EKFSlam::cbSensorLandmark, this);
        pub_landmark = nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks/kalman", 1);

        aslam::EKFSlam::initialize();
}

/// \brief Initialize
void aslam::EKFSlam::initialize()
{
        N = 3;
        init_x = true;
        init_z = true;
        last_time = ros::Time::now().toSec();

        param.X = VectorXd::Zero(N);
        param.Z = VectorXd::Zero(N);
        param.Q = MatrixXd::Zero(N, N);

        param.A = MatrixXd::Identity(N, N);
        param.H = MatrixXd::Identity(N, N);
        param.I = MatrixXd::Identity(N, N);

        param.R = MatrixXd::Identity(N, N) * EKF_KR;
        param.P = MatrixXd::Identity(N, N) * EKF_KP_ROBOT_POSE;

        param.Q(0, 0) = EKF_KQ;
        param.Q(1, 1) = EKF_KQ;
        param.Q(2, 2) = EKF_KQ;
        param.P.bottomRightCorner(N - 3, N - 3) = MatrixXd::Identity(N - 3, N - 3) * EKF_KP_LANDMARK_POSE;
}

/// \brief Odom callback
void aslam::EKFSlam::cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
        if (init_z)
                return;

        // Calculate delta time
        float delta_time = std::min(ros::Time::now().toSec() - last_time, 1.0);
        last_time = ros::Time::now().toSec();

        // Update Z and A with new measurement data
        aslam::EKFSlam::updateZandA(msg, delta_time);

        // Initialize X with Z only once
        if (init_x)
        {
                init_x = false;
                param.X = param.Z;
        }

        // SLAM
        aslam::EKFSlam::slam(msg->twist.twist.linear.x, msg->twist.twist.angular.z, delta_time);

        // Publish landmarks to visualize in Rviz
        awesome_slam_msgs::Landmarks L = convertToLandmarkMsg(N, param.X);
        pub_landmark.publish(L);
}

/// \brief Update landmarks: range and bearing from laser data
void aslam::EKFSlam::cbSensorLandmark(const awesome_slam_msgs::Landmarks::ConstPtr &msg)
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

/// \brief Update H
void aslam::EKFSlam::updateH()
{
        for (uint32_t i = 0; i < N - 3; i += 2)
        {
                float hyp = std::pow(param.X(3 + i) - param.X(0), 2) + std::pow(param.X(4 + i) - param.X(1), 2);
                float dist = std::sqrt(hyp);

                param.H(3 + i, 0) = (-param.X(3 + i) + param.X(0)) / dist;
                param.H(4 + i, 0) = -(-param.X(4 + i) + param.X(1)) / hyp;
                param.H(3 + i, 1) = (-param.X(4 + i) + param.X(1)) / dist;
                param.H(4 + i, 1) = (-param.X(3 + i) + param.X(0)) / hyp;
                param.H(4 + i, 2) = -1;
                param.H(3 + i, 3 + i) = -(-param.X(3 + i) + param.X(0)) / dist;
                param.H(3 + i, 4 + i) = -(-param.X(4 + i) + param.X(1)) / dist;
                param.H(4 + i, 3 + i) = (-param.X(4 + i) + param.X(1)) / hyp;
                param.H(4 + i, 4 + i) = -(-param.X(3 + i) + param.X(0)) / hyp;
        }
}

/// \brief Update Z and A
void aslam::EKFSlam::updateZandA(const nav_msgs::Odometry::ConstPtr &msg, const float &delta_time)
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
                        aslam::EKFSlam::updateNewLandmarkWait(data);
                        continue;
                }

                // Find nearest neighbour
                int corr_id = 0;
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
                        aslam::EKFSlam::updateNewLandmarkWait(data);
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
                aslam::EKFSlam::updateNewLandmark(new_landmark);
                new_landmark.clear();
        }

        // Update A
        if (msg->twist.twist.linear.x && msg->twist.twist.angular.z)
        {
                float delta_theta = msg->twist.twist.angular.z * delta_time;
                float r = msg->twist.twist.linear.x / msg->twist.twist.angular.z;
                param.A(0, 0) = r * (-std::cos(param.Z(2)) + std::cos(param.Z(2) + delta_theta));
                param.A(1, 0) = r * (-std::sin(param.Z(2)) + std::sin(param.Z(2) + delta_theta));
        }
}

/// \brief If point is already present in new_landmark_wait increment
///        its count else push as new landmark in new_landmark_wait
void aslam::EKFSlam::updateNewLandmarkWait(const LaserData &data)
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

void aslam::EKFSlam::updateNewLandmark(const std::vector<LaserData> &new_landmark)
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
        param.I.conservativeResizeLike(MatrixXd::Identity(N, N));
        param.A.conservativeResizeLike(MatrixXd::Identity(N, N));
        param.H.conservativeResizeLike(MatrixXd::Identity(N, N));
        param.P.conservativeResizeLike(MatrixXd::Identity(N, N) * UKF_KP_LANDMARK_POSE);
        param.R.conservativeResizeLike(MatrixXd::Identity(N, N) * UKF_KR);

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

/// \brief Perform predict and update steps
void aslam::EKFSlam::slam(const float &vx, const float &az, const float &delta_time)
{
        param.X = stateTransitionFunction(N, param.X, vx, az, delta_time);
        param.X(2) = normalizeAngle(param.X(2));
        param.P = param.A * param.P * param.A.transpose() + param.Q;

        aslam::EKFSlam::updateH();
        MatrixXd S = param.H * param.P * param.H.transpose() + param.R;
        MatrixXd K = param.P * param.H.transpose() * S.inverse();
        VectorXd Y = param.Z - measurementFunction(N, param.X);

        for (uint32_t j = 0; j < N - 1; j += 2)
        {
                Y(2 + j) = normalizeAngle(Y(2 + j));
        }

        param.X = param.X + K * Y;
        param.P = (param.I - K * param.H) * param.P;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "aslam_ekf");
        ros::Time::init();
        ros::Rate rate(FREQ);
        aslam::EKFSlam a;
        std::cerr << "[EKF] Node started!\n";

        while (ros::ok())
        {
                ros::spinOnce();
                rate.sleep();
        }
}