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
    subOdom      = nh.subscribe("/odom", 1, &aslam::UKFSlam::cbOdom, this);
    subLaser     = nh.subscribe("/laser/scan", 1, &aslam::UKFSlam::cbLaser, this);
    pubLandmarks = nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks", 1);
    
    aslam::UKFSlam::initialize();
}


/// \brief Destructor
aslam::UKFSlam::~UKFSlam()
{ 
    delete lm;
    delete param;
}


/// \brief Initialize matrices, and other vars
void aslam::UKFSlam::initialize()
{
    initX=true;
    initZwithLaser=true;
    
    param->Q = Eigen::MatrixXf::Zero(N,N);
    param->R = Eigen::MatrixXf::Identity(N,N)                           * 0.2 ;
    param->P = Eigen::MatrixXf::Identity(N,N)                           * 0.001;
    
    param->Q.block<3,3>(0,0) = Eigen::MatrixXf::Identity(3,3)           * 0.001;
    param->P.block<N-3,N-3>(3,3) = Eigen::MatrixXf::Identity(N-3,N-3)   * 100;
}


/// \brief Update landmarks: range and bearing from laser data
void aslam::UKFSlam::cbLaser(const sensor_msgs::LaserScan::ConstPtr &scan) 
{
    initZwithLaser=false; 
    lm->updateLandMarks(scan);
}


/// \brief Update Z
void aslam::UKFSlam::updateZ(const nav_msgs::Odometry::ConstPtr& msg)
{
    float theta = std::atan2(2*(msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + 
                                msg->pose.pose.orientation.x * msg->pose.pose.orientation.y), 
                                1-2*(std::pow(msg->pose.pose.orientation.z,2) + 
                                std::pow(msg->pose.pose.orientation.y,2)));

    param->Z(0) = msg->pose.pose.position.x;
    param->Z(1) = msg->pose.pose.position.y;
    param->Z(2) = normalizeAngle(theta);
    for(int i=0; i<N-3; i+=2)
    {
        param->Z(3+i) = lm->range[i/2];
        param->Z(4+i) = normalizeAngle(lm->bearing[i/2]);
    }
}


/// \brief Initialize X with Z
void aslam::UKFSlam::initXwithZ()
{
    param->X(0) = param->Z(0);
    param->X(1) = param->Z(1);
    param->X(2) = param->Z(2);
    for(int i=0; i<N-3; i+=2)
    {
        param->X(3+i) = param->Z(0)+param->Z(3+i)*std::cos(param->Z(2)+param->Z(4+i));
        param->X(4+i) = param->Z(1)+param->Z(3+i)*std::sin(param->Z(2)+param->Z(4+i));
    }
}


/// \brief Odom callback
void aslam::UKFSlam::cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(initZwithLaser) return;
    aslam::UKFSlam::updateZ(msg);

    if(initX)
    {
        initX = false;
        aslam::UKFSlam::initXwithZ();
    }

    aslam::UKFSlam::slam(msg->twist.twist.linear.x, msg->twist.twist.angular.z);
    aslam::UKFSlam::publishLandmarks();
}


/// \brief State transition function aka A
MatrixN1 aslam::UKFSlam::stateTransitionFunction(const MatrixN1& point, float vx, float az)
{
    MatrixN1 _point = point;
    if(vx && az)
    {
        float radius = vx / az;
        _point(0) += radius*(-std::sin(point(2)) + std::sin(point(2) + az));
        _point(1) += radius*( std::cos(point(2)) - std::cos(point(2) + az));
        _point(2) += az;
    }
    return _point;
}


/// \brief Measurement function aka H
MatrixN1 aslam::UKFSlam::measurementFunction(const MatrixN1& point)
{
    MatrixN1 _point = point;
    for(int i=0; i<N-3; i+=2)
    {
        _point(3+i) = std::sqrt(std::pow(point(3+i)-point(0),2) + std::pow(point(4+i)-point(1),2));
        _point(4+i) = normalizeAngle(std::atan2(point(4+i)-point(1), point(3+i)-point(0))-point(2));
    }
    return _point;
}


/// \brief Generate sigma points
/// Transform sigma points
/// Compute predict and update step
void aslam::UKFSlam::slam(float vx, float az)
{

    // Generate sigma points XX
    std::vector<MatrixN1> XX = {param->X};
    MatrixNN chol = param->P.llt().matrixL();
    for(int i=0; i<2*N; i++)
    {   
        if(i<N) { XX.push_back(param->X + (std::sqrt(N + param->lambda)*chol).block<1,N>(i,0).transpose()); }
        else    { XX.push_back(param->X - (std::sqrt(N + param->lambda)*chol).block<1,N>(i-N,0).transpose()); }
    }


    // PREDICT STEP
    std::vector<MatrixN1> YY;
    for(int i=0; i<=2*N; i++)  { YY.push_back(aslam::UKFSlam::stateTransitionFunction(XX[i], vx, az)); }
    
    param->X = param->wMean0 * YY[0];
    for(int i=1; i<=2*N; i++) { param->X += param->wRest * YY[i]; }

    param->P = param->wCov0 * (YY[0] - param->X) * (YY[0] - param->X).transpose();
    for(int i=1; i<=2*N; i++) { param->P += param->wRest * (YY[i] - param->X) * (YY[i] - param->X).transpose(); }
    
    param->P = param->P + param->Q;


    // UPDATE STEP
    std::vector<MatrixN1> ZZ;
    for(int i=0; i<=2*N; i++) { ZZ.push_back(aslam::UKFSlam::measurementFunction(YY[i])); }
    
    param->muZ = param->wMean0 * ZZ[0];
    for(int i=1; i<=2*N; i++) { param->muZ += param->wRest * ZZ[i]; }
    
    param->y = param->Z - param->muZ;
    
    param->Pz = param->wCov0 * (ZZ[0] - param->muZ) * (ZZ[0] - param->muZ).transpose();
    for(int i=1; i<=2*N; i++) { param->Pz += param->wRest * (ZZ[i] - param->muZ) * (ZZ[i] - param->muZ).transpose(); }
    
    param->Pz = param->Pz + param->R;
    
    param->K = param->wCov0 * (YY[0] - param->X) * (ZZ[0] - param->muZ).transpose();
    for(int i=1; i<=2*N; i++) { param->K += param->wRest * (YY[i] - param->X) * (ZZ[i] - param->muZ).transpose(); }
    
    param->K = param->K * param->Pz.inverse();
    
    param->X = param->X + (param->K * param->y);
    
    param->P = param->P - (param->K * param->Pz * param->K.transpose());
}


/// \brief Publish landmarks to visualize in rviz
void aslam::UKFSlam::publishLandmarks()
{
    awesome_slam_msgs::Landmarks l;
    std::vector<double> _predictedLandmarkX(LANDMARKS_COUNT), _predictedLandmarkY(LANDMARKS_COUNT);

    for(int i=0; i<N-3; i+=2)
    {
        _predictedLandmarkX[i/2] = param->X(3+i);
        _predictedLandmarkY[i/2] = param->X(4+i);
    }

    l.x = _predictedLandmarkX;
    l.y = _predictedLandmarkY;
    pubLandmarks.publish(l);
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