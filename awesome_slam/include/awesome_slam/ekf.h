#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <awesome_slam_msgs/Landmarks.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>


#define DEG2RAD 0.01745329251           // PI/180
#define PI 3.141592654                  // PI
#define LANDMARKS_COUNT 3               // Number of landmarks
#define N 9                             // LANDMARKS_COUNT*2 + 3


namespace aslam
{
    class EKFSlam
    {
        public:
            EKFSlam();

        private:
            ros::NodeHandle nh;
            ros::Subscriber subLaser, subOdom;
            ros::Publisher pubLandmarks;
            Eigen::Matrix<float,N,N> I, A, P, K, H, Q, R, S;
            Eigen::Matrix<float,N,1> X, Z, Y, W;
            std::vector<std::pair<float,float>> landmarks;
            bool onLandmark, initX, initXwithLaser;
            int landmarkStartAngle, k;

            void initialize();
            void cbLaser(const sensor_msgs::LaserScan::ConstPtr &scan);
            void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);
            float normalizeAngle(float theta);
            void updateH();
            void slam();
            void publishLandmarks();
    };
}