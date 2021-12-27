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
        sub_laser = nh.subscribe("/laser/scan", 10, &aslam::LandMarks::callback, this);
        pub_landmarks = nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks/sensor", 10);

        aslam::LandMarks::initialize();
}

/// \brief Initialize
void aslam::LandMarks::initialize()
{
        // Create sin and cos maps to optimize performance
        for (uint16_t theta = 0; theta < 360; ++theta)
        {
                sin_map.push_back(std::sin(DEG2RAD * static_cast<float>(theta)));
                cos_map.push_back(std::cos(DEG2RAD * static_cast<float>(theta)));
        }
}

/// \brief Laser callback
void aslam::LandMarks::callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
        Point p1, p2;
        p1 = bearing2pose(0, msg->ranges[0]);
        p2 = bearing2pose(359, msg->ranges[359]);

        uint16_t theta = 0;
        std::vector<Point> first_cluster;

        // IF first and last point belong to same cluster
        if (p1.distance(p2) < MIN_DIST_THRESH)
        {
                // Store them to use it later
                first_cluster.push_back(p1);
                while (theta < 360 && p1.distance(p2) < MIN_DIST_THRESH)
                {
                        p2 = bearing2pose(theta, msg->ranges[theta++]);
                        first_cluster.push_back(p2);
                        p1 = p2;
                }
        }

        // Sanity check
        assert(theta < 359);

        p1 = bearing2pose(theta, msg->ranges[theta++]);
        std::vector<Point> cluster = {p1};

        Circle circle_result;
        LaserData laser_data;
        std::vector<double> measured_x;
        std::vector<double> measured_y;

        while (theta < 360)
        {
                p2 = bearing2pose(theta, msg->ranges[theta++]);

                if (p1.distance(p2) < MIN_DIST_THRESH)
                {
                        cluster.push_back(p2);
                }
                else if (cluster.size() > MIN_CLUSTER_POINTS && circleClassification(cluster))
                {
                        // Ignore clusters with fewer points
                        if (theta == 360)
                        {
                                // Preallocate memory to speed up
                                std::vector<Point> joined_cluster;
                                joined_cluster.reserve(cluster.size() + first_cluster.size());
                                joined_cluster.insert(joined_cluster.end(), cluster.begin(), cluster.end());
                                joined_cluster.insert(joined_cluster.end(), first_cluster.begin(), first_cluster.end());
                                cluster = joined_cluster;
                        }

                        circle_result = circleFitting(cluster);
                        laser_data = circle_result.toLaserData();
                        measured_x.push_back(laser_data.range);
                        measured_y.push_back(laser_data.bearing);
                        cluster.clear();
                }
                else
                {
                        cluster.clear();
                }

                p1 = p2;
        }

        awesome_slam_msgs::Landmarks landmark;
        landmark.x = measured_x;
        landmark.y = measured_y;

        // Publish
        pub_landmarks.publish(landmark);
}

/// \brief Bearing to Pose
inline Point aslam::LandMarks::bearing2pose(const int16_t theta, const float &dist) const
{
        float x = dist * cos_map[theta];
        float y = dist * sin_map[theta];
        Point ret(x, y);
        return ret;
}

/// \brief Circle Classification algorithm
///     J. Xavier et. al., Fast line, arc/circle and leg detection from laser
///     scan data in a Player driver, ICRA 2005
bool aslam::LandMarks::circleClassification(const std::vector<Point> &points) const
{
        Point P;
        float sum = 0.0;
        int size = points.size();
        Point P1 = points.front();
        Point P2 = points.back();
        std::vector<float> angles;

        for (int i = 1; i < size - 1; i++)
        {
                P = points[i];

                // Calculate the angle using cosine law
                float c = P1.distance(P2);
                float a = P1.distance(P);
                float b = P2.distance(P);

                float angle = std::acos((a * a + b * b - c * c) / (2 * a * b));
                angles.push_back(angle);
                sum += angle;
        }

        float variance = 0.0;
        float mean = sum / (size - 2);

        for (const float &val : angles)
        {
                variance += pow(val - mean, 2);
        }

        float std = sqrt(variance / 5.0);

        if (std < STD && mean > MIN_MEAN && mean < MAX_MEAN)
        {
                return true;
        }

        return false;
}

/// \brief Circular Fitting algorithm
///     A. Al-Sharadqah and N. Chernov, Error Analysis for Circle Fitting
///     Algorithms, Electronic Journal of Statistics (2009), Volume 3 p
///     886-911
Circle aslam::LandMarks::circleFitting(const std::vector<Point> &points_)
{
        std::vector<Point> points;
        points = points_;

        float x_mean = 0.0;
        float y_mean = 0.0;
        float z_mean = 0.0;

        for (const Point &pt : points)
        {
                x_mean += pt(0);
                y_mean += pt(1);
        }

        x_mean = x_mean / static_cast<float>(points.size());
        y_mean = y_mean / static_cast<float>(points.size());

        // Shift the centroid of data points
        for (Point &pt : points)
        {
                pt.assign(pt(0) - x_mean, pt(1) - y_mean);
        }

        // Compute z and z_mean
        std::vector<float> z;
        for (const Point &pt : points)
        {
                z.push_back(pt(0) * pt(0) + pt(1) * pt(1));
        }

        for (const float &z_i : z)
        {
                z_mean += z_i;
        }

        z_mean = z_mean / static_cast<float>(z.size());

        // Form data matrix Z
        MatrixXd Z;
        Z = MatrixXd::Zero(points.size(), 4);
        int size = points.size();

        for (int i = 0; i < size; i++)
        {
                VectorXd vec(4);
                vec << z[i], points[i](0), points[i](1), 1;
                Z.row(i) = vec.transpose();
        }

        MatrixXd M;
        Eigen::Matrix4d H, H_inv;
        M = (1 / static_cast<float>(points.size())) * Z.transpose() * Z;

        H << 8 * z_mean, 0, 0, 2, 0, 1, 0, 0, 0, 0, 1, 0, 2, 0, 0, 0;
        H_inv << 0, 0, 0, (1.0 / 2.0), 0, 1, 0, 0, 0, 0, 1, 0, (1.0 / 2.0), 0, 0, -2 * z_mean;

        MatrixXd A;
        VectorXd singularValues;
        Eigen::JacobiSVD<MatrixXd> svd(Z, Eigen::ComputeThinU | Eigen::ComputeThinV);

        singularValues = svd.singularValues();

        auto U = svd.matrixU();
        auto V = svd.matrixV();

        if (singularValues(3) > 10e-12)
        {
                float smallest_id = 0;
                float smallest_ev = 99999;

                VectorXd ev;
                MatrixXd Y, Q, A_star;
                Eigen::Matrix<double, 4, 4> sigma;

                sigma = singularValues.array().matrix().asDiagonal();
                Y = V * sigma * V.transpose();
                Q = Y * H_inv * Y;

                // Find the eigenvector corresponding to the smallest positive eigenvalue
                Eigen::SelfAdjointEigenSolver<MatrixXd> es(Q);

                ev = es.eigenvalues();

                for (int i = 0; i < ev.size(); i++)
                {
                        if (ev[i] > 0 && ev[i] < smallest_ev)
                        {
                                smallest_id = i;
                                smallest_ev = ev[i];
                        }
                }

                A_star = es.eigenvectors().col(smallest_id);
                A = Y.colPivHouseholderQr().solve(A_star);
        }
        else
        {
                A = V.col(3);
        }

        float a = (-A(1)) / (2 * A(0));
        float b = (-A(2)) / (2 * A(0));
        Circle ret(a + x_mean, b + y_mean);

        return ret;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "aslam_sensor_landmark");
        ros::Time::init();
        ros::Rate rate(FREQ);
        aslam::LandMarks a;
        std::cerr << "[SENSOR LANDMARK] Node started!\n";

        while (ros::ok())
        {
                ros::spinOnce();
                rate.sleep();
        }
}
