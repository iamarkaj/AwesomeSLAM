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

#include "sensor_landmark.h"

/// \brief Constructor
aslam::LandMarks::LandMarks() : nh(ros::NodeHandle())
{
    subLaser = nh.subscribe("/laser/scan", 1, &aslam::LandMarks::callback, this);
    pubLandmarks = nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks/sensor", 1);

    aslam::LandMarks::initialize();
}

/// \brief Initialize
void aslam::LandMarks::initialize()
{
    threshold = 0.5;

    // Create sin and cos maps to optimize performance
    for (uint16_t theta = 0; theta < 360; ++theta)
    {
        sinMap.push_back(std::sin(DEG2RAD * static_cast<float>(theta)));
        cosMap.push_back(std::cos(DEG2RAD * static_cast<float>(theta)));
    }
}

/// \brief Laser callback
void aslam::LandMarks::callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    uint16_t i = 0;

    // Create a new cluster
    std::vector<Point> cluster;

    while (i < 360)
    {
        // Clear existing cluster
        cluster.clear();

        // Put the i th point into a new vector
        cluster.push_back(Point(bearing2pose(i, msg->ranges.at(i))));
        i++;

        while (i < 360)
        {
            // If distance between two points are less than
            // the threshold, then they are the same cluster
            float dist =
                Point(bearing2pose(i - 1, msg->ranges.at(i - 1))).distance(Point(bearing2pose(i, msg->ranges.at(i))));
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

    if (pointClusters.front().front().distance(pointClusters.back().back()) < threshold)
    {
        pointClusters.back().insert(pointClusters.back().end(), pointClusters.front().begin(),
                                    pointClusters.front().end());
        pointClusters.erase(pointClusters.begin());
    }

    pointClusters.erase(std::remove_if(pointClusters.begin(), pointClusters.end(),
                                       [](const std::vector<Point> &x) { return x.size() <= 3; }),
                        pointClusters.end());

    std::vector<double> measuredLMx;
    std::vector<double> measuredLMy;

    for (auto clu : pointClusters)
    {
        if (circleClassification(clu))
        {
            Circle circleResult = circleFitting(clu);
            LaserData data = circleResult.toLaserData();

            measuredLMx.push_back(data.range);
            measuredLMy.push_back(data.bearing);
        }
    }

    pointClusters.clear();

    awesome_slam_msgs::Landmarks L;
    L.x = measuredLMx;
    L.y = measuredLMy;

    // Publish
    pubLandmarks.publish(L);
}

/// \brief Bearing to Pose
inline Point aslam::LandMarks::bearing2pose(const int16_t &theta, const float &dist) const
{
    float x = dist * cosMap[theta];
    float y = dist * sinMap[theta];
    Point ret(x, y);
    return ret;
}

/// \brief Circle Classification algorithm
///     J. Xavier et. al., Fast line, arc/circle and leg detection from laser
///     scan data in a Player driver, ICRA 2005
bool aslam::LandMarks::circleClassification(const std::vector<Point> &points) const
{
    float sum = 0.0;
    int size = points.size();
    Point P1 = points.front();
    Point P2 = points.back();
    std::vector<float> angles;

    for (int i = 1; i < size - 1; i++)
    {
        Point P = points.at(i);

        // Calculate the angle using cosine law
        float c = P1.distance(P2);
        float a = P1.distance(P);
        float b = P2.distance(P);

        float angle = std::acos((a * a + b * b - c * c) / (2 * a * b));
        angles.push_back(angle);
        sum += angle;
    }

    float std = 0.0;
    float variance = 0.0;
    float mean = sum / (size - 2);

    for (const float &val : angles)
    {
        variance += pow(val - mean, 2);
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
///     A. Al-Sharadqah and N. Chernov, Error Analysis for Circle Fitting
///     Algorithms, Electronic Journal of Statistics (2009), Volume 3 p
///     886-911
Circle aslam::LandMarks::circleFitting(std::vector<Point> &points)
{
    float xMean = 0.0;
    float yMean = 0.0;
    float zMean = 0.0;

    for (const Point &pt : points)
    {
        xMean += pt(0);
        yMean += pt(1);
    }

    xMean = xMean / static_cast<float>(points.size());
    yMean = yMean / static_cast<float>(points.size());

    // Shift the centroid of data points
    for (Point &pt : points)
    {
        pt.assign(pt(0) - xMean, pt(1) - yMean);
    }

    // Compute z and zMean
    std::vector<float> z;
    for (Point &pt : points)
    {
        z.push_back(pt(0) * pt(0) + pt(1) * pt(1));
    }

    for (const float &z_i : z)
    {
        zMean += z_i;
    }

    zMean = zMean / static_cast<float>(z.size());

    // Form data matrix Z
    MatrixXd Z = MatrixXd::Zero(points.size(), 4);
    int size = points.size();

    for (int i = 0; i < size; i++)
    {
        VectorXd vec(4);
        vec << z.at(i), points.at(i)(0), points.at(i)(1), 1;
        Z.row(i) = vec.transpose();
    }

    MatrixXd M = (1 / static_cast<float>(points.size())) * Z.transpose() * Z;
    Eigen::Matrix4d H, H_inv;

    H << 8 * zMean, 0, 0, 2, 0, 1, 0, 0, 0, 0, 1, 0, 2, 0, 0, 0;

    H_inv << 0, 0, 0, (1.0 / 2.0), 0, 1, 0, 0, 0, 0, 1, 0, (1.0 / 2.0), 0, 0, -2 * zMean;

    Eigen::JacobiSVD<MatrixXd> svd(Z, Eigen::ComputeThinU | Eigen::ComputeThinV);
    VectorXd singularValues = svd.singularValues();
    MatrixXd A;

    auto U = svd.matrixU();
    auto V = svd.matrixV();

    if (singularValues(3) > 10e-12)
    {
        Eigen::Matrix<double, 4, 4> sigma = singularValues.array().matrix().asDiagonal();
        MatrixXd Y = V * sigma * V.transpose();

        // Form Q matrix
        MatrixXd Q = Y * H_inv * Y;

        // Find the eigenvector corresponding to the smallest positive eigenvalue
        Eigen::SelfAdjointEigenSolver<MatrixXd> es(Q);
        VectorXd ev = es.eigenvalues();

        float smallest_ev = 99999;
        float smallest_id = 0;

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

    float a = (-A(1)) / (2 * A(0));
    float b = (-A(2)) / (2 * A(0));
    Circle ret(a + xMean, b + yMean);

    return ret;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aslam_sensor_landmark");
    ros::Time::init();
    ros::Rate rate(1);
    aslam::LandMarks a;
    std::cerr << "[SENSOR LANDMARK] Node started!\n";

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}
