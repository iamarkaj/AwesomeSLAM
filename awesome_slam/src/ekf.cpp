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
    subOdom      = nh.subscribe("/odom", 1, &aslam::EKFSlam::cbOdom, this);
    subLaser     = nh.subscribe("/laser/scan", 1, &aslam::EKFSlam::cbLaser, this);
    pubLandmarks = nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks", 1);
    
    aslam::EKFSlam::initialize();
}


/// \brief Destructor
aslam::EKFSlam::~EKFSlam()
{ 
    delete param;
    delete lm;
}


/// \brief Initialize matrices, and other vars
void aslam::EKFSlam::initialize()
{
    initX=true;
    initZwithLaser=true;
    
    param->W = Eigen::MatrixXf::Zero(N,1);
    param->Q = Eigen::MatrixXf::Zero(N,N);
    param->A = Eigen::MatrixXf::Identity(N,N);
    param->H = Eigen::MatrixXf::Identity(N,N);
    param->I = Eigen::MatrixXf::Identity(N,N);
    param->R = Eigen::MatrixXf::Identity(N,N)*0.2;
    param->P = Eigen::MatrixXf::Identity(N,N)*0.001;
    
    param->W(0) = 0.001;
    param->W(1) = 0.001;
    param->W(2) = 0.001;

    param->P.block<N-3,N-3>(3,3) = Eigen::MatrixXf::Identity(N-3,N-3)*100;
    param->Q.block<3,3>(0,0) = Eigen::MatrixXf::Identity(3,3)*0.001;
}


/// \brief Update landmarks: range and bearing from laser data
void aslam::EKFSlam::cbLaser(const sensor_msgs::LaserScan::ConstPtr &scan) 
{
    initZwithLaser=false; 
    lm->updateLandMarks(scan);
}


/// \brief Update Z and A
void aslam::EKFSlam::updateZandA(const nav_msgs::Odometry::ConstPtr& msg)
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
    
    if(msg->twist.twist.linear.x && msg->twist.twist.angular.z)
    {
        float radius = msg->twist.twist.linear.x / msg->twist.twist.angular.z;
        param->A(0,0) = radius*(-std::cos(param->Z(2)) + std::cos(param->Z(2) + msg->twist.twist.angular.z));
        param->A(1,0) = radius*(-std::sin(param->Z(2)) + std::sin(param->Z(2) + msg->twist.twist.angular.z));
    }
}


/// \brief Initialize X with Z
void aslam::EKFSlam::initXwithZ()
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
void aslam::EKFSlam::cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(initZwithLaser) return;
    aslam::EKFSlam::updateZandA(msg);

    if(initX)
    {
        initX = false;
        aslam::EKFSlam::initXwithZ();
    }

    aslam::EKFSlam::slam();
    aslam::EKFSlam::publishLandmarks();
}


/// \brief Update H
void aslam::EKFSlam::updateH()
{
    for(int i=0; i<N-3; i+=2)
    {
        float var = std::pow(param->X(3+i)-param->X(0),2) + std::pow(param->X(4+i)-param->X(1),2);
        float dist = std::sqrt(var);
        param->H(3+i,0) = (-param->X(3+i)+param->X(0))/dist;
        param->H(4+i,0) = -(-param->X(4+i)+param->X(1))/var;
        param->H(3+i,1) = (-param->X(4+i)+param->X(1))/dist;
        param->H(4+i,1) = (-param->X(3+i)+param->X(0))/var;
        param->H(4+i,2) = -1;
        param->H(3+i,3+i) = -(-param->X(3+i)+param->X(0))/dist;
        param->H(3+i,4+i) = -(-param->X(4+i)+param->X(1))/dist;
        param->H(4+i,3+i) = (-param->X(4+i)+param->X(1))/var;
        param->H(4+i,4+i) = -(-param->X(3+i)+param->X(0))/var;
    }
}


/// \brief Perform predict and update steps
void aslam::EKFSlam::slam()
{
    param->X = param->A * param->X + param->W;
    param->P = param->A * param->P * param->A.transpose() + param->Q;

    aslam::EKFSlam::updateH();
    param->S = param->H * param->P * param->H.transpose() + param->R;
    param->K = param->P * param->H.transpose() * param->S.inverse();
    param->Y = param->Z - (param->H * param->X);
    param->X = param->X + param->K * param->Y;
    param->P = (param->I - param->K * param->H) * param->P;
}


/// \brief Publish landmarks to visualize in rviz
void aslam::EKFSlam::publishLandmarks()
{
    awesome_slam_msgs::Landmarks l;
    std::vector<double> _predictedLandmarkX(3), _predictedLandmarkY(3);

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