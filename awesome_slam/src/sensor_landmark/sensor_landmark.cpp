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
aslam::LandMarks::LandMarks() : nh(ros::NodeHandle()) {
  subLaser = nh.subscribe("/laser/scan", 1, &aslam::LandMarks::callback, this);
  pubLandmarks =
      nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks/sensor", 1);

  aslam::LandMarks::initialize();
}

/// \brief Initialize
void aslam::LandMarks::initialize() {
  // Create sin and cos maps to optimize performance
  for (uint16_t theta = 0; theta < 360; ++theta) {
    sinMap.push_back(std::sin(DEG2RAD * static_cast<float>(theta)));
    cosMap.push_back(std::cos(DEG2RAD * static_cast<float>(theta)));
  }
}

/// \brief Laser callback
void aslam::LandMarks::callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  Point p1, p2;
  uint16_t i = 0;
  LaserData laserData;
  Circle circleResult;
  std::vector<Point> cluster;
  std::vector<Point> firstCluster;
  std::vector<double> measuredLMx;
  std::vector<double> measuredLMy;

  p1 = bearing2pose(0, msg->ranges[0]);
  p2 = bearing2pose(359, msg->ranges[359]);

  // First and last point belong to same cluster
  if (p1.distance(p2) < 0.5) {
    // Store them to use it later
    firstCluster.push_back(p1);

    while (i < 360 && p1.distance(p2) < 0.5) {
      p2 = bearing2pose(i, msg->ranges[i++]);
      firstCluster.push_back(p2);
      p1 = p2;
    }
  }

  // Sanity check
  assert(i < 359 && "[ERROR]");

  p1 = bearing2pose(i, msg->ranges[i++]);
  cluster.push_back(p1);

  while (i < 360) {
    p2 = bearing2pose(i, msg->ranges[i++]);

    if (p1.distance(p2) < 0.5) {
      cluster.push_back(p2);
    } else if (cluster.size()) {
      // Ignore clusters with fewer points
      if (cluster.size() > 3 && circleClassification(cluster)) {
        if (i == 360) {
          // Preallocate memory to speed up
          std::vector<Point> lastAndFirstCluster;
          lastAndFirstCluster.reserve(cluster.size() + firstCluster.size());
          lastAndFirstCluster.insert(lastAndFirstCluster.end(), cluster.begin(),
                                     cluster.end());
          lastAndFirstCluster.insert(lastAndFirstCluster.end(),
                                     firstCluster.begin(), firstCluster.end());

          circleResult = circleFitting(lastAndFirstCluster);
        } else {
          circleResult = circleFitting(cluster);
        }

        laserData = circleResult.toLaserData();
        measuredLMx.push_back(laserData.range);
        measuredLMy.push_back(laserData.bearing);
      }

      cluster.clear();
    }

    p1 = p2;
  }

  awesome_slam_msgs::Landmarks L;
  L.x = measuredLMx;
  L.y = measuredLMy;

  // Publish
  pubLandmarks.publish(L);
}

/// \brief Bearing to Pose
inline Point aslam::LandMarks::bearing2pose(const int16_t theta,
                                            const float &dist) const {
  float x = dist * cosMap[theta];
  float y = dist * sinMap[theta];
  Point ret(x, y);
  return ret;
}

/// \brief Circle Classification algorithm
///     J. Xavier et. al., Fast line, arc/circle and leg detection from laser
///     scan data in a Player driver, ICRA 2005
bool aslam::LandMarks::circleClassification(
    const std::vector<Point> &points) const {
  Point P;
  float sum = 0.0;
  int size = points.size();
  Point P1 = points.front();
  Point P2 = points.back();
  std::vector<float> angles;

  for (int i = 1; i < size - 1; i++) {
    P = points[i];

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

  for (const float &val : angles) {
    variance += pow(val - mean, 2);
  }

  std = sqrt(variance / 5.0);

  if (std < 0.4 && mean > 1.5 && mean < 3.0) {
    return true;
  }

  return false;
}

/// \brief Circular Fitting algorithm
///     A. Al-Sharadqah and N. Chernov, Error Analysis for Circle Fitting
///     Algorithms, Electronic Journal of Statistics (2009), Volume 3 p
///     886-911
Circle aslam::LandMarks::circleFitting(const std::vector<Point> &points_) {
  std::vector<Point> points;
  points = points_;

  float xMean = 0.0;
  float yMean = 0.0;
  float zMean = 0.0;

  for (const Point &pt : points) {
    xMean += pt(0);
    yMean += pt(1);
  }

  xMean = xMean / static_cast<float>(points.size());
  yMean = yMean / static_cast<float>(points.size());

  // Shift the centroid of data points
  for (Point &pt : points) {
    pt.assign(pt(0) - xMean, pt(1) - yMean);
  }

  // Compute z and zMean
  std::vector<float> z;
  for (const Point &pt : points) {
    z.push_back(pt(0) * pt(0) + pt(1) * pt(1));
  }

  for (const float &z_i : z) {
    zMean += z_i;
  }

  zMean = zMean / static_cast<float>(z.size());

  // Form data matrix Z
  MatrixXd Z = MatrixXd::Zero(points.size(), 4);
  int size = points.size();

  for (int i = 0; i < size; i++) {
    VectorXd vec(4);
    vec << z[i], points[i](0), points[i](1), 1;
    Z.row(i) = vec.transpose();
  }

  MatrixXd M = (1 / static_cast<float>(points.size())) * Z.transpose() * Z;
  Eigen::Matrix4d H, H_inv;

  H << 8 * zMean, 0, 0, 2, 0, 1, 0, 0, 0, 0, 1, 0, 2, 0, 0, 0;
  H_inv << 0, 0, 0, (1.0 / 2.0), 0, 1, 0, 0, 0, 0, 1, 0, (1.0 / 2.0), 0, 0,
      -2 * zMean;

  MatrixXd A;
  VectorXd singularValues;
  Eigen::JacobiSVD<MatrixXd> svd(Z, Eigen::ComputeThinU | Eigen::ComputeThinV);

  singularValues = svd.singularValues();

  auto U = svd.matrixU();
  auto V = svd.matrixV();

  if (singularValues(3) > 10e-12) {
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

    for (int i = 0; i < ev.size(); i++) {
      if (ev[i] > 0 && ev[i] < smallest_ev) {
        smallest_id = i;
        smallest_ev = ev[i];
      }
    }

    A_star = es.eigenvectors().col(smallest_id);
    A = Y.colPivHouseholderQr().solve(A_star);
  } else {
    A = V.col(3);
  }

  float a = (-A(1)) / (2 * A(0));
  float b = (-A(2)) / (2 * A(0));
  Circle ret(a + xMean, b + yMean);

  return ret;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aslam_sensor_landmark");
  ros::Time::init();
  ros::Rate rate(1);
  aslam::LandMarks a;
  std::cerr << "[SENSOR LANDMARK] Node started!\n";

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
