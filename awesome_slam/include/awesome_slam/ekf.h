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

#ifndef EKF_SLAM_H
#define EKF_SLAM_H

#include <awesome_slam/common.h>

struct parameters {
    MatrixNN I;
    MatrixNN A;
    MatrixNN P;
    MatrixNN K;
    MatrixNN H;
    MatrixNN Q;
    MatrixNN R;
    MatrixNN S;
    MatrixN1 X;
    MatrixN1 Z;
    MatrixN1 Y;
    MatrixN1 W;
};


namespace aslam {
class EKFSlam {
    public:
        EKFSlam();
        ~EKFSlam();

    private:
        ros::NodeHandle nh;
        ros::Subscriber subOdom;
        ros::Subscriber subLaser;
        ros::Publisher pubLandmarks;

        LandMarks* lm = new LandMarks;
        parameters* param = new parameters;

        bool initX; 
        bool initZwithLaser;

        void slam();
        void updateH();
        void initialize();
        void initXwithZ();
        void publishLandmarks();
        void updateZandA(const nav_msgs::Odometry::ConstPtr& msg);
        void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);
        void cbLaser(const sensor_msgs::LaserScan::ConstPtr &scan);
    };
}

#endif // EKF_SLAM_H