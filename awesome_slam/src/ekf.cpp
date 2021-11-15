#include <awesome_slam/ekf.h>


aslam::EKFSlam::EKFSlam():nh(ros::NodeHandle())
{ 
    pubLandmarks = nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks", 1);
    subLaser     = nh.subscribe("/laser/scan", 1, &aslam::EKFSlam::cbLaser, this);
    subOdom      = nh.subscribe("/odom", 1, &aslam::EKFSlam::cbOdom, this);
    aslam::EKFSlam::initialize();
}



/// \brief Initialize matrices, and other vars
void aslam::EKFSlam::initialize()
{
    onLandmark = false;
    landmarkStartAngle = 0, k = 0;
    initX = true, initXwithLaser = true;
    
    A = Eigen::MatrixXf::Identity(N,N);
    W = Eigen::MatrixXf::Zero(N,1);
    W(0) = 0.001;
    W(1) = 0.001;
    W(2) = 0.001;

    P = Eigen::MatrixXf::Identity(N,N)*0.00001;
    P.block<LANDMARKS_COUNT*2,LANDMARKS_COUNT*2>(3,3) = Eigen::MatrixXf::Identity(LANDMARKS_COUNT*2,LANDMARKS_COUNT*2)*1000;

    Q = Eigen::MatrixXf::Zero(N,N);
    Q.block<3,3>(0,0) = Eigen::MatrixXf::Identity(3,3)*0.001;

    H = Eigen::MatrixXf::Identity(N,N);
    I = Eigen::MatrixXf::Identity(N,N);
    R = Eigen::MatrixXf::Identity(N,N)*0.2;

    for(int i=0; i<LANDMARKS_COUNT; i++){ landmarks.push_back({0.0,0.0}); }
}



/// \brief TODO : Revisit and fix the naive solution
/// Update landmarks: range and bearing from laser data
void aslam::EKFSlam::cbLaser(const sensor_msgs::LaserScan::ConstPtr &scan) 
{
    initXwithLaser = false; // Updated landmarks with initial laser data
    int tmp = 0;
    for(int i=0; i<360; i++) 
    {
        if(scan->ranges[i]<=scan->range_max && !onLandmark) 
        {
            landmarkStartAngle = i;
            onLandmark = true;
        } 
        else if(scan->ranges[i]>scan->range_max && onLandmark) 
        {
            tmp = ((i-1)<landmarkStartAngle) ? int((i-1+landmarkStartAngle-360)>>1) : int((i-1+landmarkStartAngle)>>1);
            landmarks[k].first = scan->ranges[tmp];
            landmarks[k].second = DEG2RAD*tmp;
            k++;
            if(k>=LANDMARKS_COUNT) { k=0; }
            onLandmark = false;
        }
    }
}



/// \brief Odom callback
void aslam::EKFSlam::cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    /// \brief Update landmarks with initial laser data first
    ////////////////////////////////////////////////////////////
    if(initXwithLaser) return;


    /// \brief Update Z
    ////////////////////////////////////////////////////////////
    float theta = std::atan2(2*(msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + 
                                msg->pose.pose.orientation.x * msg->pose.pose.orientation.y), 
                                1-2*(std::pow(msg->pose.pose.orientation.z,2) + 
                                std::pow(msg->pose.pose.orientation.y,2)));
    
    Z(0) = msg->pose.pose.position.x;
    Z(1) = msg->pose.pose.position.y;
    Z(2) = (theta<0) ? theta+ADJUST_ANGLE : theta;

    for(int i=0; i<LANDMARKS_COUNT*2; i+=2)
    {
        Z(3+i) = landmarks[i/2].first;
        Z(4+i) = landmarks[i/2].second;
    }


    /// \brief Initialize X with odom and laser data, will run only once
    ////////////////////////////////////////////////////////////
    if(initX)
    {
        initX = false;
        X = Eigen::MatrixXf::Zero(N,1);
        X(0) = Z(0);
        X(1) = Z(1);
        X(2) = Z(2);

        for(int i=0; i<LANDMARKS_COUNT*2; i+=2)
        {
            X(3+i) = Z(0)+Z(3+i)*std::cos(Z(2)+Z(4+i));
            X(4+i) = Z(1)+Z(3+i)*std::sin(Z(2)+Z(4+i));
        }
    }

    /// \brief Update A
    ////////////////////////////////////////////////////////////
    float R = msg->twist.twist.linear.x / msg->twist.twist.angular.z;
    A(0,0) = R*(-std::cos(Z(2)) + std::cos(Z(2) + msg->twist.twist.angular.z));
    A(0,1) = R*(-std::sin(Z(2)) + std::sin(Z(2) + msg->twist.twist.angular.z));


    ////////////////////////////////////////////////////////////
    aslam::EKFSlam::slam();


    ////////////////////////////////////////////////////////////
    aslam::EKFSlam::publishLandmarks();
}



/// \brief Update H
void aslam::EKFSlam::updateH()
{
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



/// \brief Perform predict and update steps
void aslam::EKFSlam::slam()
{
    X = A * X + W;
    P = A * P * A.transpose() + Q;

    aslam::EKFSlam::updateH();
    S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();
    Y = Z - (H * X);
    X = X + K * Y;
    P = (I - K * H) * P;
}



/// \brief Publish landmarks to visualize in rviz
void aslam::EKFSlam::publishLandmarks()
{
    awesome_slam_msgs::Landmarks l;
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



int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_ekf");
    ros::Time::init();
    ros::Rate rate(1);
    aslam::EKFSlam a;
    
    while(ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }
}