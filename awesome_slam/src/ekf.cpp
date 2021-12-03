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

#include <awesome_slam/ekf.h>


/// \brief Constructor
aslam::EKFSlam::EKFSlam():nh(ros::NodeHandle())
{ 
    subOdom = nh.subscribe("/odom", 1, &aslam::EKFSlam::cbOdom, this);
    subLaser = nh.subscribe("/laser/scan", 1, &aslam::EKFSlam::cbLaser, this);
    pubLandmarks = nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks", 1);
    
    aslam::EKFSlam::initialize();
}


/// \brief Initialize
void aslam::EKFSlam::initialize()
{
    initX = true;
    initZwithLaser = true;
    lastTime = ros::Time::now().toSec();
    
    param.X = VectorXd::Zero(N);
    param.Z = VectorXd::Zero(N);
    param.Q = MatrixXd::Zero(N,N);

    param.A = MatrixXd::Identity(N,N);
    param.H = MatrixXd::Identity(N,N);
    param.I = MatrixXd::Identity(N,N);
    
    param.R = MatrixXd::Identity(N,N) * 0.2;
    param.P = MatrixXd::Identity(N,N) * 0.001;
    
    param.Z(3) = -1.0;
    param.Q(0,0) = 0.001;
    param.Q(1,1) = 0.001;
    param.Q(2,2) = 0.001;
    param.P.bottomRightCorner(N-3,N-3) = MatrixXd::Identity(N-3,N-3) * 10000.0;
}


/// \brief Update landmarks: range and bearing from laser data
void aslam::EKFSlam::cbLaser(const sensor_msgs::LaserScan::ConstPtr &msg) 
{
    initZwithLaser = false; 
    lm.updateLandMarks(msg);
}


/// \brief Update Z and A
void aslam::EKFSlam::updateZandA(const nav_msgs::Odometry::ConstPtr& msg, const double& deltaTime)
{
    double theta = std::atan2(2*(msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + 
                                msg->pose.pose.orientation.x * msg->pose.pose.orientation.y), 
                                1-2*(std::pow(msg->pose.pose.orientation.z,2) + 
                                std::pow(msg->pose.pose.orientation.y,2)));

    param.Z(0) = msg->pose.pose.position.x;
    param.Z(1) = msg->pose.pose.position.y;
    param.Z(2) = theta;

    // Data Association
    int size = lm.range.size();
    // Initial
    if(param.Z(3) == -1.0)
    {
        for(int i=0; i<size*2; i+=2)
        {
            param.Z(3+i) = lm.range[i/2];
            param.Z(4+i) = lm.bearing[i/2];
        }
    }
    else
    {
        for(int i=0; i<size; ++i)
        {
            int corrIdx = 0;
            lm.bearing[i] = normalizeAngle(lm.bearing[i]);
            Point newLandmarkPos(param.Z(0) + lm.range[i] * std::cos(param.Z(2) + lm.bearing[i]),
                                 param.Z(1) + lm.range[i] * std::sin(param.Z(2) + lm.bearing[i]));
            Point oldLandmarkPos(param.X(3), param.X(4));
            double mindist = distance(newLandmarkPos, oldLandmarkPos);
            for(int j=2; j<N-3; j+=2)
            {
                oldLandmarkPos << param.X(3+j), param.X(4+j);
                double dist = distance(newLandmarkPos, oldLandmarkPos);
                if(dist < mindist)
                {
                    corrIdx = j;
                    mindist = dist;
                }
            }
            param.Z(3+corrIdx) = lm.range[i];
            param.Z(4+corrIdx) = lm.bearing[i];
        }
    }
    
    // Update A
    if(msg->twist.twist.linear.x && msg->twist.twist.angular.z)
    {
        double delTheta = msg->twist.twist.angular.z * deltaTime;
        double radius = msg->twist.twist.linear.x / msg->twist.twist.angular.z;
        param.A(0,0) = radius * (-std::cos(param.Z(2)) + std::cos(param.Z(2) + delTheta));
        param.A(1,0) = radius * (-std::sin(param.Z(2)) + std::sin(param.Z(2) + delTheta));
    }
}


/// \brief Initialize X with Z
void aslam::EKFSlam::initXwithZ()
{
    param.X(0) = param.Z(0);
    param.X(1) = param.Z(1);
    param.X(2) = param.Z(2);
    for(int i=0; i<N-3; i+=2)
    {
        param.X(3+i) = param.Z(0) + param.Z(3+i) * std::cos(param.Z(2) + param.Z(4+i));
        param.X(4+i) = param.Z(1) + param.Z(3+i) * std::sin(param.Z(2) + param.Z(4+i));
    }
}


/// \brief Odom callback
void aslam::EKFSlam::cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(initZwithLaser) return;

    // Calculate delta time
    double deltaTime = std::min(ros::Time::now().toSec() - lastTime, 1.0);
    lastTime = ros::Time::now().toSec();

    // Update Z and A with new measurement data
    aslam::EKFSlam::updateZandA(msg, deltaTime);

    // Call initXwithZ function only once
    if(initX)
    {
        initX = false;
        aslam::EKFSlam::initXwithZ();
    }

    // SLAM
    aslam::EKFSlam::slam(msg->twist.twist.linear.x, msg->twist.twist.angular.z, deltaTime);

    // Publish landmarks to visualize in Rviz
    aslam::EKFSlam::publishLandmarks();
}


/// \brief Update H
void aslam::EKFSlam::updateH()
{
    for(int i=0; i<N-3; i+=2)
    {
        double hyp = std::pow(param.X(3+i)-param.X(0),2) + std::pow(param.X(4+i)-param.X(1),2);
        double dist = std::sqrt(hyp);
        param.H(3+i,0) = (-param.X(3+i)+param.X(0))/dist;
        param.H(4+i,0) = -(-param.X(4+i)+param.X(1))/hyp;
        param.H(3+i,1) = (-param.X(4+i)+param.X(1))/dist;
        param.H(4+i,1) = (-param.X(3+i)+param.X(0))/hyp;
        param.H(4+i,2) = -1;
        param.H(3+i,3+i) = -(-param.X(3+i)+param.X(0))/dist;
        param.H(3+i,4+i) = -(-param.X(4+i)+param.X(1))/dist;
        param.H(4+i,3+i) = (-param.X(4+i)+param.X(1))/hyp;
        param.H(4+i,4+i) = -(-param.X(3+i)+param.X(0))/hyp;
    }
}


/// \brief Perform predict and update steps
void aslam::EKFSlam::slam(const double& vx, const double& az, const double& deltaTime)
{   
    param.X = stateTransitionFunction(param.X, vx, az, deltaTime);
    param.X(2) = normalizeAngle(param.X(2));
    param.P = param.A * param.P * param.A.transpose() + param.Q;

    aslam::EKFSlam::updateH();
    param.S = param.H * param.P * param.H.transpose() + param.R;
    param.K = param.P * param.H.transpose() * param.S.inverse();
    param.Y = param.Z - measurementFunction(param.X);

    for(int j=0; j<N-1; j+=2)
    {
        param.Y(2+j) = normalizeAngle(param.Y(2+j));
    }

    param.X = param.X + param.K * param.Y;
    param.P = (param.I - param.K * param.H) * param.P;
}


/// \brief Publish landmarks to visualize in rviz
void aslam::EKFSlam::publishLandmarks()
{
    std::vector<double> predictedX(LANDMARKS_COUNT);
    std::vector<double> predictedY(LANDMARKS_COUNT);

    for(int i=0; i<N-3; i+=2)
    {
        predictedX[i/2] = param.X(3+i);
        predictedY[i/2] = param.X(4+i);
    }

    awesome_slam_msgs::Landmarks L;
    L.x = predictedX;
    L.y = predictedY;

    pubLandmarks.publish(L);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_ekf");
    ros::Time::init();
    ros::Rate rate(1);
    aslam::EKFSlam a;
    std::cerr << "[EKF] Node started!\n";
    
    while(ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }
}