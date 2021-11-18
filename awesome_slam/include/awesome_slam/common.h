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

#ifndef ASLAM_COMMON_H
#define ASLAM_COMMON_H

#include <ros/ros.h>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <awesome_slam/common.h>
#include <awesome_slam_msgs/Landmarks.h>

#define PI 3.141592654          /// <PI
#define DEG2RAD 0.01745329251   /// <PI/180
#define LANDMARKS_COUNT 3       /// <Number of landmarks
#define N 9                     /// <LANDMARKS_COUNT*2 + 3

typedef Eigen::Matrix<float,N,N> MatrixNN;
typedef Eigen::Matrix<float,N,1> MatrixN1;


enum laserRange {
    onLandmark,
    offLandmark
};


class LandMarks {
private:
    uint8_t idx;
    laserRange laser;
    uint8_t landmarkStartAngle; 

public:
    std::vector<float> range;
    std::vector<float> bearing;

    LandMarks() 
    {
        idx = -1;
        laser = offLandmark;
        landmarkStartAngle=0;
        for(int i=0; i<LANDMARKS_COUNT; i++)
        {
            range.push_back(0.0);
            bearing.push_back(0.0);
        }
    }

    void updateLandMarks(const sensor_msgs::LaserScan::ConstPtr &scan) 
    {
        int tmp = 0;
        for(int i=0; i<360; i++) 
        {
            if(scan->ranges[i]<=scan->range_max && laser==offLandmark) 
            {
                landmarkStartAngle = i;
                laser = onLandmark;
            } 
            else if(scan->ranges[i]>scan->range_max && laser==onLandmark) 
            {
                if(i<landmarkStartAngle && landmarkStartAngle>360-i) 
                    tmp = landmarkStartAngle+i-360;
                else if(i<landmarkStartAngle && landmarkStartAngle<360-i) 
                    tmp = landmarkStartAngle-i+360; 
                else 
                    tmp = landmarkStartAngle+i; 

                tmp >>= 1;
                range[++idx] = scan->ranges[tmp];
                bearing[idx] = DEG2RAD*tmp;
                laser = offLandmark;
                
                if(idx==LANDMARKS_COUNT) idx = 0; 
            }
        }
    }
};


/// \brief Normalize angle
float normalizeAngle(float theta)
{
    theta = std::fmod(theta, 2*PI); // move in range  0  to 2PI
    if(theta>PI) theta -= 2*PI;     // move in range -PI to PI
    return theta;
}

#endif // ASLAM_COMMON_H