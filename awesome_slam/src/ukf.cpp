#include <awesome_slam/ukf.h>


aslam::UKFSlam::UKFSlam():nh(ros::NodeHandle())
{ 
    pubLandmarks = nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks", 1);
    subLaser     = nh.subscribe("/laser/scan", 100, &aslam::UKFSlam::cbLaser, this);
    subOdom      = nh.subscribe("/odom", 1, &aslam::UKFSlam::cbOdom, this);
    aslam::UKFSlam::initialize();
}



/// \brief Initialize matrices, and other vars
void aslam::UKFSlam::initialize()
{
    onLandmark = false;
    landmarkStartAngle = 0, t = 0;
    initX = true, initXwithLaser = true;

    a = 0.2;
    b = 2.0;
    k = 3-N;

    lambda = std::pow(a,2)*(N+k)-N;
    wMean0 = lambda/(N+lambda);
    wCov0  = lambda/(N+lambda)+1-std::pow(a,2)+b;
    wRest  = 1/(2*(N+lambda));
    
    P = Eigen::MatrixXf::Identity(N,N)*0.001;
    R = Eigen::MatrixXf::Identity(N,N)*0.02;
    
    Q = Eigen::MatrixXf::Zero(N,N);
    Q.block<3,3>(0,0) = Eigen::MatrixXf::Identity(3,3)*0.000001;
    
    for(int i=0; i<LANDMARKS_COUNT; i++){ landmarks.push_back({0.0,0.0}); }
}



/// \brief TODO : Revisit and fix the naive solution
/// Update landmarks: range and bearing from laser data
void aslam::UKFSlam::cbLaser(const sensor_msgs::LaserScan::ConstPtr &scan) 
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
            landmarks[t].first = scan->ranges[tmp];
            landmarks[t].second = aslam::UKFSlam::normalizeAngle(DEG2RAD*tmp);
            t++;
            if(t>=LANDMARKS_COUNT) { t=0; }
            onLandmark = false;
        }
    }
}



/// \brief Normalize angle
float aslam::UKFSlam::normalizeAngle(float theta)
{
    theta = std::fmod(theta, 2*PI);         // move in range  0  to 2PI
    if(theta>PI) { theta = theta - 2*PI; }  // move in range -PI to PI
    return theta;
}



/// \brief Odom callback
void aslam::UKFSlam::cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
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
    Z(2) = aslam::UKFSlam::normalizeAngle(theta);

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
        X(0) = Z(0);
        X(1) = Z(1);
        X(2) = Z(2);

        for(int i=0; i<LANDMARKS_COUNT*2; i+=2)
        {
            X(3+i) = Z(0)+Z(3+i)*std::cos(Z(2)+Z(4+i));
            X(4+i) = Z(1)+Z(3+i)*std::sin(Z(2)+Z(4+i));
        }
    }


    ////////////////////////////////////////////////////////////
    aslam::UKFSlam::slam(msg->twist.twist.linear.x, msg->twist.twist.angular.z);

    
    ////////////////////////////////////////////////////////////
    aslam::UKFSlam::publishLandmarks();
}



/// \brief State transition function aka A
Eigen::Matrix<float,N,1> aslam::UKFSlam::stateTransitionFunction(const Eigen::Matrix<float,N,1>& point, float vx, float az)
{
    Eigen::Matrix<float,N,1> _point = point;
    float R = vx / az;
    _point(0) += R*(-std::sin(point(2)) + std::sin(point(2) + az));
    _point(1) += R*( std::cos(point(2)) - std::cos(point(2) + az));
    _point(2) += az;
    return _point;
}



/// \brief Measurement function aka H
Eigen::Matrix<float,N,1> aslam::UKFSlam::measurementFunction(const Eigen::Matrix<float,N,1>& point)
{
    Eigen::Matrix<float,N,1> _point = point;
    for(int i=0; i<LANDMARKS_COUNT*2; i+=2)
    {
        _point(3+i) = std::sqrt(std::pow(point(3+i)-point(0),2) + std::pow(point(4+i)-point(1),2));
        _point(4+i) = std::atan2(point(3+i)-point(0), point(4+i)-point(1))-point(2);
    }
    return _point;
}



/// \brief Generate sigma points
/// transform sigma points
/// compute predict and update step
void aslam::UKFSlam::slam(float vx, float az)
{
    /// \brief Generate sigma points XX
    ////////////////////////////////////////////////////////////
    std::vector<Eigen::Matrix<float,N,1>> XX;
    Eigen::Matrix<float,N,N> chol = P.llt().matrixL();
    for(int i=0; i<2*N; i++)
    {   
        if(i<N) { XX.push_back(X + (std::sqrt(N + lambda)*chol).block<1,9>(i,0).transpose()); }
        else    { XX.push_back(X - (std::sqrt(N + lambda)*chol).block<1,9>(i-N,0).transpose()); }
    }


    /// \brief PREDICT STEP
    ////////////////////////////////////////////////////////////
    std::vector<Eigen::Matrix<float,N,1>> YY;
    for(int i=0; i<2*N; i++)  { YY.push_back(aslam::UKFSlam::stateTransitionFunction(XX[i], vx, az)); }
    
    X = wMean0 * YY[0];
    for(int i=1; i<2*N; i++) { X += wRest * YY[i]; }

    P = wCov0 * (YY[0] - X) * (YY[0] - X).transpose();
    for(int i=1; i<2*N; i++) { P += wRest * (YY[i] - X) * (YY[i] - X).transpose(); }
    
    P = P + Q;



    /// \brief UPDATE STEP
    ////////////////////////////////////////////////////////////
    std::vector<Eigen::Matrix<float,N,1>> ZZ;
    for(int i=0; i<2*N; i++) { ZZ.push_back(aslam::UKFSlam::measurementFunction(YY[i])); }
    
    muZ = wMean0 * ZZ[0];
    for(int i=1; i<2*N; i++) { muZ += wRest * ZZ[i]; }
    
    y = Z - muZ;
    
    Pz = wCov0 * (ZZ[0] - muZ) * (ZZ[0] - muZ).transpose();
    for(int i=1; i<2*N; i++) { Pz += wRest * (ZZ[i] - muZ) * (ZZ[i] - muZ).transpose(); }
    
    Pz = Pz + R;
    
    K = wCov0 * (YY[0] - X) * (ZZ[0] - muZ).transpose();
    for(int i=1; i<2*N; i++) { K += wRest * (YY[i] - X) * (ZZ[i] - muZ).transpose(); }
    
    K = K * Pz.inverse();
    
    X = X + (K * y);
    
    P = P - (K * Pz * K.transpose());
}



/// \brief Publish landmarks to visualize in rviz
void aslam::UKFSlam::publishLandmarks()
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
    ros::init(argc, argv, "slam_ukf");
    ros::Time::init();
    ros::Rate rate(1);
    aslam::UKFSlam a;
    
    while(ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }
}