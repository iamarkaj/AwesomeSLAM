#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <awesome_slam/Landmarks.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#define LASER_MAX_RANGE 3.5
#define DEG2RAD 0.01745329251
#define ADJUST_ANGLE 0.10966227112
#define LANDMARKS_COUNT 3
#define N 9

namespace aslam
{
    class EKFSlam
    {
        public:
            EKFSlam();

        private:
            ros::NodeHandle nh;
            ros::Subscriber subLaser, subOdom, subTeleop;
            ros::Publisher pubLandmarks;
            Eigen::Matrix<float,N,N> I, F, P, K, H, Q, R, S;
            Eigen::Matrix<float,N,1> X, Z, B, Y;
            Eigen::Matrix<float,1,1> U;
            std::vector<std::pair<float,float>> landMarks;
            float vel;
            bool onLandmark;
            int landmarkStartAngle, k;

            void initialize();
            void updateH();
            void updateB();
            void cbLaser(const sensor_msgs::LaserScan::ConstPtr &scan);
            void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);
            void cbTeleop(const geometry_msgs::TwistConstPtr& msg);
            void slam();
            void publishLandmarks();
    };
}