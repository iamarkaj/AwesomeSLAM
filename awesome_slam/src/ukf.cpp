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


#include <awesome_slam/ukf.h>


/// \brief Constructor
aslam::UKFSlam::UKFSlam():nh(ros::NodeHandle())
{ 
    subOdom = nh.subscribe("/odom", 1, &aslam::UKFSlam::cbOdom, this);
    subLaser = nh.subscribe("/laser/scan", 1, &aslam::UKFSlam::cbLaser, this);
    pubLandmarks = nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks", 1);
    
    aslam::UKFSlam::initialize();
}


/// \brief Initialize
void aslam::UKFSlam::initialize()
{
    initX = true;
    initZwithLaser = true;
    lastTime = ros::Time::now().toSec();
    
    param.X = VectorXd::Zero(N);
    param.Z = VectorXd::Zero(N);
    param.Q = MatrixXd::Zero(N,N);

    param.R = MatrixXd::Identity(N,N) * 0.2;
    param.P = MatrixXd::Identity(N,N) * 0.001;
    
    param.Z(3) = -1.0;
    param.Q(0,0) = 0.001;
    param.Q(1,1) = 0.001;
    param.Q(2,2) = 0.001;
    param.P.bottomRightCorner(N-3,N-3) = MatrixXd::Identity(N-3,N-3) * 1.0;
}


/// \brief Update landmarks: range and bearing from laser data
void aslam::UKFSlam::cbLaser(const sensor_msgs::LaserScan::ConstPtr &msg) 
{
    initZwithLaser = false; 
    lm.updateLandMarks(msg);
}


/// \brief Update Z
void aslam::UKFSlam::updateZ(const nav_msgs::Odometry::ConstPtr& msg)
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
}


/// \brief Initialize X with Z
void aslam::UKFSlam::initXwithZ()
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
void aslam::UKFSlam::cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(initZwithLaser) return;

    // Calculate delta time
    double deltaTime = std::min(ros::Time::now().toSec() - lastTime, 1.0);
    lastTime = ros::Time::now().toSec();

    // Update Z with new measurement data
    aslam::UKFSlam::updateZ(msg);

    // Call initXwithZ function only once
    if(initX)
    {
        initX = false;
        aslam::UKFSlam::initXwithZ();
    }

    // SLAM
    aslam::UKFSlam::slam(msg->twist.twist.linear.x, msg->twist.twist.angular.z, deltaTime);

    // Publish landmarks to visualize in Rviz
    aslam::UKFSlam::publishLandmarks();
}


/// \brief SLAM
void aslam::UKFSlam::slam(const double& vx, const double& az, const double& deltaTime)
{
    VectorXd Xaug = VectorXd(N+2);
    MatrixXd Paug = MatrixXd(N+2, N+2);

    // Create sigma point matrix
    MatrixXd XsigAug = MatrixXd(N+2, 2*N+5);

    // Create augmented mean state
    Xaug.head(N) = param.X;
    Xaug(N) = 0.0;
    Xaug(N+1) = 0.0;

    // Create augmented covariance matrix
    Paug.fill(0.0);
    Paug.topLeftCorner(N,N) = param.P;
    Paug(N,N) = param.std_a*param.std_a;
    Paug(N+1,N+1) = param.std_yaw*param.std_yaw;

    // Create square root matrix
    MatrixXd L = Paug.llt().matrixL();

    // Create augmented sigma points
    XsigAug.col(0) = Xaug;
    double w = std::sqrt(param.lambda+N+2);
    for(int i=0; i<N+2; ++i) 
    {
        XsigAug.col(i+1)   = Xaug + w * L.col(i);
        XsigAug.col(i+3+N) = Xaug - w * L.col(i);
    }

    // Create matrix with predicted sigma points as columns
    MatrixXd XsigPred = MatrixXd(N, 2*N+5);
    for(int i=0; i<2*N+5; ++i) 
    {
        XsigPred.col(i)= stateTransitionFunction(XsigAug.col(i), vx, az, deltaTime);
        XsigPred.col(i)(2) = normalizeAngle(XsigPred.col(i)(2));
    }


    // Predicted state mean
    param.X.fill(0.0);
    for (int i=0; i<2*N+5; ++i) 
    {
        param.X += param.weights(i) * XsigPred.col(i);
    }

    // Predicted state covariance matrix
    param.P.fill(0.0);
    for (int i=0; i<2*N+5; ++i) 
    {
        // State difference
        VectorXd Xdiff = XsigPred.col(i) - param.X;
        Xdiff(2) = normalizeAngle(Xdiff(2));

        param.P += param.weights(i) * Xdiff * Xdiff.transpose();
    }

    // Add noise
    param.P = param.P + param.Q;

    // Transform sigma points into measurement space
    MatrixXd Zsig = MatrixXd(N, 2*N+5);
    for(int i=0; i<2*N+5; ++i) 
    {
        Zsig.col(i) = measurementFunction(XsigPred.col(i));
    }

    // Mean predicted measurement
    VectorXd Zpred = VectorXd::Zero(N);
    for(int i=0; i<2*N+5; ++i) 
    {
        Zpred += param.weights(i) * Zsig.col(i);
    }

    // Normalize
    for(int j=0; j<N-1; j+=2)
    {
        Zpred(2+j) = normalizeAngle(Zpred(2+j));
    }

    // Innovation covariance matrix S
    MatrixXd S = MatrixXd::Zero(N,N);
    for(int i=0; i<2*N+5; ++i) 
    {
        // Residual
        VectorXd Zdiff = Zsig.col(i) - Zpred;
        for(int j=0; j<N-1; j+=2)
        {
            Zdiff(2+j) = normalizeAngle(Zdiff(2+j));
        }

        S = S + param.weights(i) * Zdiff * Zdiff.transpose();
    }

    // Add measurement noise covariance matrix
    S = S + param.R;

    // Calculate cross correlation matrix
    MatrixXd Tc = MatrixXd::Zero(N,N);
    for(int i=0; i<2*N+5; ++i) 
    {
        // Residual
        VectorXd Zdiff = Zsig.col(i) - Zpred;
        for(int j=0; j<N-1; j+=2)
        {
            Zdiff(2+j) = normalizeAngle(Zdiff(2+j));
        }

        // State difference
        VectorXd Xdiff = XsigPred.col(i) - param.X;
        Xdiff(2) = normalizeAngle(Xdiff(2));

        Tc = Tc + param.weights(i) * Xdiff * Zdiff.transpose();
    }

    // Kalman gain K
    MatrixXd K = Tc * S.inverse();

    // Residual
    VectorXd Zdiff = param.Z - Zpred;

    for(int j=0; j<N-1; j+=2)
    {
        Zdiff(2+j) = normalizeAngle(Zdiff(2+j));
    }

    // Update state mean and covariance matrix
    param.X = param.X + K * Zdiff;

    param.P = param.P - K * S * K.transpose();
}


/// \brief Publish landmarks to visualize in rviz
void aslam::UKFSlam::publishLandmarks()
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
    ros::init(argc, argv, "slam_ukf");
    ros::Time::init();
    ros::Rate rate(1);
    aslam::UKFSlam a;
    std::cerr << "[UKF] Node started!\n";
    
    while(ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }
}