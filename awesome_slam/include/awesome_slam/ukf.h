#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <awesome_slam_msgs/Landmarks.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Cholesky>


#define DEG2RAD 0.01745329251           // PI/180
#define PI 3.141592654                  // PI
#define LANDMARKS_COUNT 3               // Number of landmarks
#define N 9                             // LANDMARKS_COUNT*2 + 3


namespace aslam
{
    class UKFSlam
    {
        public:
            UKFSlam();

        private:
            ros::NodeHandle nh;
            ros::Subscriber subLaser, subOdom;
            ros::Publisher pubLandmarks;
            Eigen::Matrix<float,N,N> P, K, Pz, Q, R;
            Eigen::Matrix<float,N,1> X, Z, muZ, y;
            std::vector<std::pair<float,float>> landmarks;
            bool onLandmark, initX, initXwithLaser;
            float a, b, k, lambda, wMean0, wCov0, wRest;
            int landmarkStartAngle, t;

            void initialize();
            void cbLaser(const sensor_msgs::LaserScan::ConstPtr &scan);
            void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);
            float normalizeAngle(float theta);
            Eigen::Matrix<float,N,1> stateTransitionFunction(const Eigen::Matrix<float,N,1>& point, float vx, float az);
            Eigen::Matrix<float,N,1> measurementFunction(const Eigen::Matrix<float,N,1>& point);
            void slam(float vx, float vy);
            void publishLandmarks();
    };
}