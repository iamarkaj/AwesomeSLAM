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

using namespace std;
class Aslam
{
    public:
        Aslam():nh(ros::NodeHandle())
        {
            initialize();
            pubLandmarks = nh.advertise<awesome_slam::Landmarks>("out/landmarks", 1);
            subLaser = nh.subscribe("/laser/scan", 1, &Aslam::cbLaser, this);
            subOdom = nh.subscribe("/odom", 1, &Aslam::cbOdom, this);
            subTeleop = nh.subscribe("/cmd_vel", 1, &Aslam::cbTeleop, this);
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber subLaser, subOdom, subTeleop;
        ros::Publisher pubLandmarks;
        Eigen::Matrix<float,N,N> I, F, P, K, H, Q, R, S;
        Eigen::Matrix<float,N,1> X, Z, B, Y;
        Eigen::Matrix<float,1,1> U;
        std::vector<std::pair<float,float>> landMarks;
        float vel;
        bool onLandmark = false;
        int landmarkStartAngle=0, k=0;


        void initialize()
        {
            vel = 0.0;

            I = Eigen::MatrixXf::Identity(N,N);
            X = Eigen::MatrixXf::Identity(N,1);
            Z = X;

            P = 0.02*I;

            F = I;
            H = I;
            B = X;
            U << 1;

            Q = 0.001*I;
            R = I;
            R.block<3,3>(0,0) = Eigen::MatrixXf::Identity(3,3)*0.2;
            R.block<LANDMARKS_COUNT*2,LANDMARKS_COUNT*2>(3,3) = Eigen::MatrixXf::Identity(LANDMARKS_COUNT*2,LANDMARKS_COUNT*2)*0.5;

            for(int i=0; i<LANDMARKS_COUNT; i++){ landMarks.push_back({2.0,2.0}); }
        }


        void updateH()
        {
            H = I;
            for(int i=0; i<LANDMARKS_COUNT*2; i+=2)
            {
                float var = std::pow(X(3+i)-X(0),2) + std::pow(X(4+i)-X(1),2), dist = std::sqrt(var);
                H(3+i,0) = (-X(3+i)+X(0))/dist;
                H(4+i,0) = -(-X(4+i)+X(1))/var;
                H(3+i,1) = (-X(4+i)+X(1))/dist;
                H(4+i,1) = (-X(3+i)+X(0))/var;
                H(4+i,2) = -1;
                H(3+i,3+i) = -(-X(3+i)+X(0))/dist;
                H(3+i,4+i) = -(-X(4+i)+X(1))/dist;
                H(4+i,3+i) = (-X(4+i)+X(1))/var;
                H(4+i,4+i) = -(-X(3+i)+X(0))/var;
            }
        }


        void updateB()
        {
            B = Eigen::MatrixXf::Zero(N,1);
            B(0) = -vel*std::sin(X(2));
            B(1) = vel*std::cos(X(2));
        }


        void cbLaser(const sensor_msgs::LaserScan::ConstPtr &scan) 
        {
            /*
            TODO : Revisit and fix the naive solution
            */

            int tmp=0;
            for(int i=0; i<360; i++) 
            {
                if(scan->ranges[i]<=LASER_MAX_RANGE && !onLandmark) 
                {
                    landmarkStartAngle = i;
                    onLandmark = true;
                } 
                else if(scan->ranges[i]>LASER_MAX_RANGE && onLandmark == true) 
                {
                    tmp = ((i-1)<landmarkStartAngle) ? int((i-1+landmarkStartAngle-360)>>1) : int((i-1+landmarkStartAngle)>>1);
                    landMarks[k].first = scan->ranges[tmp];
                    landMarks[k].second = DEG2RAD*tmp;
                    k++;
                    if(k>=LANDMARKS_COUNT) { k=0; }
                    onLandmark = false;
                }
            }
        }


        void cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
        {
            // Update Measurement matrix
            Z(0) = msg->pose.pose.position.x;
            Z(1) = msg->pose.pose.position.y;

            double theta = std::atan2(2*(msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + 
                        msg->pose.pose.orientation.x * msg->pose.pose.orientation.y), 
                        1-2*(std::pow(msg->pose.pose.orientation.z,2) + 
                        std::pow(msg->pose.pose.orientation.y,2)));
            
            Z(2) = (theta<0) ? theta+ADJUST_ANGLE : theta;

            for(int i=0; i<LANDMARKS_COUNT; i++)
            {
                Z(3+i) = landMarks[i].first;
                Z(4+i) = landMarks[i].first;
            }


            // EKF SLAM
            slam();

            // Publish Landmarks
            publishLandmarks();
        }


        void cbTeleop(const geometry_msgs::TwistConstPtr& msg)
        {
            vel = msg->linear.x;
        }


        void slam()
        {
            updateB();
            X = F * X + B * U;
            P = F * P * F.transpose() + Q;

            updateH();
            S = H * P * H.transpose() + R;
            K = P * H.transpose() * S.inverse();
            Y = Z - (H * X);
            X = X + K * Y;
            P = (I - K * H) * P;
        }


        void publishLandmarks()
        {
            awesome_slam::Landmarks l;
            std::vector<double> _predictedLandmarkX(3), _predictedLandmarkY(3);
            for(int i=0; i<LANDMARKS_COUNT*2; i+=2)
            {
                _predictedLandmarkX[i/2] = X(3+i);
                _predictedLandmarkY[i/2] = X(4+i);
            }
            l.x = _predictedLandmarkX;
            l.y = _predictedLandmarkY;
            pubLandmarks.publish(l);
        }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_ekf");
    ros::Time::init();
    ros::Rate rate(1);
    std::cerr << "Starting..\n";
    Aslam a;
    
    while(ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }
}