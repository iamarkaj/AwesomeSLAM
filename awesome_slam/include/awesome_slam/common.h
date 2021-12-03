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
#include <algorithm>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <awesome_slam/common.h>
#include <awesome_slam_msgs/Landmarks.h>


using Eigen::MatrixXd;
using Eigen::VectorXd;


typedef Eigen::Vector2d Point;


#define PI 3.141592654         /// <PI
#define DEG2RAD 0.01745329251  /// <PI/180
#define LANDMARKS_COUNT 4      /// <Number of landmarks
#define N 11                   /// <2*LANDMARKS_COUNT + 3


struct Circle
{
    Point center;
    Circle(const double& x, const double& y) : center(x, y) {}
};


/// \brief Normalize angle
inline double normalizeAngle(const double& theta)
{
    double ret = theta;
    ret = std::fmod(ret, 2*PI); // move in range  0  to 2PI
    if(ret>PI) ret -= 2*PI;
    if(ret<-PI) ret += 2*PI;
    return ret;
}


/// \brief calculate the distance between two points
inline double distance(const Point& p1, const Point& p2)
{
    double ret = std::sqrt(std::pow((p1(0) - p2(0)), 2) + std::pow((p1(1) - p2(1)), 2));
    return ret;
}


class LandMarks {
private:
    float threshold;
    std::vector<double> sinMap;
    std::vector<double> cosMap;
    std::vector<std::vector<Point>> pointClusters;


    /// \brief Bearing to Pose
    inline Point bearing2pose(const int theta, const float& dist)
    {
        double x = dist*cosMap[theta];
        double y = dist*sinMap[theta];
        Point ret(x, y);
        return ret;
    }


    /// \brief Circle Classification algorithm
    ///     J. Xavier et. al., Fast line, arc/circle and leg detection from laser
    ///     scan data in a Player driver, ICRA 2005
    bool circleClassification(const std::vector<Point>& points)
    {
        double sum = 0.0;
        int size = points.size();
        Point P1 = points.front();
        Point P2 = points.back();
        std::vector<double> angles;
        

        for (int i=1; i<size-1; i++)
        {
            Point P = points.at(i);

            // Calculate the angle using cosine law
            double c = distance(P1, P2);
            double a = distance(P1, P);
            double b = distance(P2, P);
            double angle = std::acos((std::pow(a, 2)+std::pow(b, 2)-std::pow(c, 2))/(2*a*b));
            angles.push_back(angle);
            sum += angle;
        }

        double std = 0.0;
        double variance = 0.0;
        double mean = sum/(size-2);

        for (const double& val : angles)
        {
            variance += pow(val-mean, 2);
        }

        variance = variance / 5.0;
        std = sqrt(variance);
        
        if (std < 0.4 && mean > 1.5 && mean < 3.0)
        {
            return true;
        }
        
        return false;
    }


    /// \brief Circular Fitting algorithm
    ///     A. Al-Sharadqah and N. Chernov, Error Analysis for Circle Fitting Algorithms,
    ///     Electronic Journal of Statistics (2009), Volume 3 p 886-911
    Circle circleFitting(std::vector<Point> points)
    {
        double xMean = 0.0;
        double yMean = 0.0;
        double zMean = 0.0;

        for (const Point& pt : points)
        {
            xMean += pt(0);
            yMean += pt(1);
        }
        xMean = xMean / static_cast<double>(points.size());
        yMean = yMean / static_cast<double>(points.size());

        // Shift the centroid of data points
        for (Point& pt : points)
        {
            pt(0) -= xMean;
            pt(1) -= yMean;
        }

        // Compute z and zMean
        std::vector<double> z;
        for (Point& pt : points)
        {
            z.push_back(std::pow(pt(0), 2) + std::pow(pt(1), 2));
        }

        for (const double& z_i : z)
        {
            zMean += z_i;
        }
        zMean = zMean / static_cast<double>(z.size());

        // Form data matrix Z
        MatrixXd Z = MatrixXd::Zero(points.size(), 4);
        int _N = points.size(); 

        for (int i=0; i<_N; i++)
        {
            VectorXd vec(4);
            vec << z.at(i), points.at(i)(0), points.at(i)(1), 1;
            Z.row(i) = vec.transpose();
        }

        MatrixXd M = (1/static_cast<double>(points.size()))*Z.transpose()*Z;
        Eigen::Matrix4d H, H_inv;

        H << 8 * zMean, 0, 0, 2,
            0, 1, 0, 0,
            0, 0, 1, 0,
            2, 0, 0, 0;

        H_inv << 0, 0, 0, (1.0/2.0),
            0, 1, 0, 0,
            0, 0, 1, 0,
            (1.0/2.0), 0, 0, -2*zMean;

        Eigen::JacobiSVD<MatrixXd> svd(Z, Eigen::ComputeThinU | Eigen::ComputeThinV);
        VectorXd singularValues = svd.singularValues();
        MatrixXd A;

        auto U = svd.matrixU();
        auto V = svd.matrixV();

        if (singularValues(3) > 10e-12)
        {
            Eigen::Matrix<double, 4, 4> sigma = singularValues.array().matrix().asDiagonal();
            MatrixXd Y = V*sigma*V.transpose();

            // Form Q matrix
            MatrixXd Q = Y * H_inv * Y;

            // Find the eigenvector corresponding to the smallest positive eigenvalue
            Eigen::SelfAdjointEigenSolver<MatrixXd> es(Q);
            VectorXd ev = es.eigenvalues();

            double smallest_ev = 99999;
            double smallest_id = 0;

            for (int i = 0; i < ev.size(); i++)
            {
                if (ev[i] > 0 && ev[i] < smallest_ev)
                {
                    smallest_ev = ev[i];
                    smallest_id = i;
                }
            }

            MatrixXd A_star = es.eigenvectors().col(smallest_id);
            A = Y.colPivHouseholderQr().solve(A_star);
        }
        else
        {
            A = V.col(3);
        }

        double a = (-A(1))/(2*A(0));
        double b = (-A(2))/(2*A(0));
        Circle ret(a+xMean, b+yMean);

        return ret;
    }


public:    
    std::vector<double> range;
    std::vector<double> bearing;


    LandMarks() 
    {
        threshold = 0.5;

        // To optimize performance create sin and cos maps
        for(int theta=0; theta<360; theta++)
        {
            sinMap.push_back(std::sin(DEG2RAD*static_cast<double>(theta)));
            cosMap.push_back(std::cos(DEG2RAD*static_cast<double>(theta)));
        }
    }


    void updateLandMarks(const sensor_msgs::LaserScan::ConstPtr& msg) 
    {   
        int i = 0;
        while (i<360)
        {
            // Create a new cluster
            std::vector<Point> cluster;

            // Put the i th point into a new vector
            cluster.push_back(Point(bearing2pose(i, msg->ranges.at(i))));
            i++;

            while (i<360)
            {
                // If distance between two points are less than the threshold, then they are the same cluster
                double dist = distance(Point(bearing2pose(i-1, msg->ranges.at(i-1))),
                                       Point(bearing2pose(i, msg->ranges.at(i))));
                if (dist < threshold)
                {
                    cluster.push_back(Point(bearing2pose(i, msg->ranges.at(i))));
                    i++;
                }
                else
                {
                    break;
                }
            }

            // Insert the vector into pointClusters
            pointClusters.push_back(cluster);
        }

        if (distance(pointClusters.front().front(), pointClusters.back().back()) < threshold)
        {
            pointClusters.back().insert(pointClusters.back().end(), pointClusters.front().begin(), pointClusters.front().end());
            pointClusters.erase(pointClusters.begin());
        }

        pointClusters.erase(std::remove_if(
                        pointClusters.begin(), pointClusters.end(),
                        [](const std::vector<Point>& x) { return x.size() <= 3; }), pointClusters.end());

        // Reset landmarks
        range.clear();
        bearing.clear();

        for (auto clu : pointClusters)
        {
            if (circleClassification(clu))
            {
                Circle circleResult = circleFitting(clu);

                double r = std::sqrt(std::pow(circleResult.center(0), 2) + std::pow(circleResult.center(1), 2));
                double b = std::atan2(circleResult.center(1), circleResult.center(0));

                range.push_back(r);
                bearing.push_back(b);
            }
        }
        pointClusters.clear();
    }
};


/// \brief State transition function aka f
VectorXd stateTransitionFunction(const VectorXd& point, const double& vx, const double& az, const double& deltaTime)
{
    VectorXd _point = point.head(N);   

    // Avoid division by 0
    if(std::fabs(az) > 0.001)
    {
        double radius = vx / az;
        _point(0) += radius*(-std::sin(point(2)) + std::sin(point(2) + az*deltaTime));
        _point(1) += radius*( std::cos(point(2)) - std::cos(point(2) + az*deltaTime));
    }
    else
    {
        _point(0) += vx * deltaTime * std::cos(point(2));
        _point(1) += vx * deltaTime * std::sin(point(2));
    }
    _point(2) += az * deltaTime;

    // Add noise only for augmented matrix
    if (point.size() > N)
    {
        _point(0) += 0.5 * deltaTime * deltaTime * point(N) * std::cos(point(2));
        _point(1) += 0.5 * deltaTime * deltaTime * point(N) * std::sin(point(2));
        _point(2) += 0.5 * deltaTime * deltaTime * az;
    }

    return _point;
}


/// \brief Measurement function aka h
VectorXd measurementFunction(const VectorXd& point)
{
    VectorXd _point = point;
    for(int i=0; i<N-3; i+=2)
    {
        _point(3+i) = std::sqrt(std::pow(point(3+i)-point(0),2) + std::pow(point(4+i)-point(1),2));
        _point(4+i) = std::atan2(point(4+i)-point(1), point(3+i)-point(0))-point(2);
    }
    return _point;
}


#endif // ASLAM_COMMON_H