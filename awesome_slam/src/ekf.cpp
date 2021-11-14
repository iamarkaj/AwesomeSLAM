#include <awesome_slam/ekf.h>


aslam::EKFSlam::EKFSlam():nh(ros::NodeHandle())
{ 
    pubLandmarks = nh.advertise<awesome_slam_msgs::Landmarks>("out/landmarks", 1);
    subLaser     = nh.subscribe("/laser/scan", 1, &aslam::EKFSlam::cbLaser, this);
    subOdom      = nh.subscribe("/odom", 1, &aslam::EKFSlam::cbOdom, this);
    subTeleop    = nh.subscribe("/cmd_vel", 1, &aslam::EKFSlam::cbTeleop, this);
    aslam::EKFSlam::initialize();
}


void aslam::EKFSlam::initialize()
{
    vel = 0.0;
    onLandmark = false;
    landmarkStartAngle = 0, k = 0;

    I = Eigen::MatrixXf::Identity(N,N);
    X = Eigen::MatrixXf::Identity(N,1);

    Z = X;
    B = X;
    F = I;
    H = I;
    R = I;
    U << 1;

    P = 0.02*I;
    Q = 0.001*I;
    
    R.block<3,3>(0,0) = Eigen::MatrixXf::Identity(3,3)*0.1;
    R.block<LANDMARKS_COUNT*2,LANDMARKS_COUNT*2>(3,3) = Eigen::MatrixXf::Identity(LANDMARKS_COUNT*2,LANDMARKS_COUNT*2)*0.2;
    for(int i=0; i<LANDMARKS_COUNT; i++){ landMarks.push_back({2.0,2.0}); }
}


void aslam::EKFSlam::updateH()
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


void aslam::EKFSlam::updateB()
{
    B = Eigen::MatrixXf::Zero(N,1);
    B(0) = -vel*std::sin(X(2));
    B(1) = vel*std::cos(X(2));
}


void aslam::EKFSlam::cbLaser(const sensor_msgs::LaserScan::ConstPtr &scan) 
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
        else if(scan->ranges[i]>LASER_MAX_RANGE && onLandmark) 
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


void aslam::EKFSlam::cbOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    ////////////////////////////////////////////////////////////
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
        Z(4+i) = landMarks[i].second;
    }


    ////////////////////////////////////////////////////////////
    aslam::EKFSlam::slam();


    ////////////////////////////////////////////////////////////
    aslam::EKFSlam::publishLandmarks();
}


void aslam::EKFSlam::cbTeleop(const geometry_msgs::TwistConstPtr& msg)
{
    vel = msg->linear.x;
}


void aslam::EKFSlam::slam()
{
    aslam::EKFSlam::updateB();
    X = F * X + B * U;
    P = F * P * F.transpose() + Q;

    aslam::EKFSlam::updateH();
    S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();
    Y = Z - (H * X);
    X = X + K * Y;
    P = (I - K * H) * P;
}


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