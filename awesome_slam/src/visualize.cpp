#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <awesome_slam_msgs/Landmarks.h>


class ASlamVisualize
{
    public:
        ASlamVisualize():nh(ros::NodeHandle())
        {
            subLandmark = nh.subscribe("out/landmarks", 100, &ASlamVisualize::cbLandmarks, this);
            pubMarker   = nh.advertise<visualization_msgs::MarkerArray>("out/marker", 100);
            pubMap      = nh.advertise<nav_msgs::OccupancyGrid>("out/map", 100);
        }
        
    private:
        ros::NodeHandle nh;
        ros::Subscriber subLandmark;
        ros::Publisher pubMarker, pubMap;
        visualization_msgs::Marker* marker;
        nav_msgs::OccupancyGrid* grid;
        geometry_msgs::Pose* p;


        void cbLandmarks(const awesome_slam_msgs::LandmarksConstPtr& msg)
        {
            ////////////////////////////////////////////////////////////
            visualization_msgs::MarkerArray markerArray;
            for (int i=0; i<msg->x.size(); i++)
            {
                visualization_msgs::Marker _marker = createMarker(msg->x[i], msg->y[i], i);
                markerArray.markers.push_back(_marker);
            }
            pubMarker.publish(markerArray);

            ////////////////////////////////////////////////////////////
            // drawMap();
        }


        visualization_msgs::Marker createMarker(double x, double y, int id)
        {
            visualization_msgs::Marker marker;
            marker.id = id;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0.1;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();
            return marker;
        }


        // void drawMap()
        // {
        //     nav_msgs::OccupancyGrid* grid(new nav_msgs::OccupancyGrid());
        //     geometry_msgs::Pose* p(new geometry_msgs::Pose());
        //     grid->header.frame_id = "/map";
        //     grid->header.stamp = ros::Time::now();
        //     grid->info.map_load_time = ros::Time::now();
        //     grid->info.resolution = 1.0;
        //     grid->info.width = 2;
        //     grid->info.height = 2;
        //     p->position.x = 0.0;
        //     p->position.y = 0.0;
        //     p->position.z = 0.0;
        //     p->orientation.x = 0.0;
        //     p->orientation.y = 0.0;
        //     p->orientation.z = 0.0;
        //     p->orientation.w = 0.0;
        //     grid->info.origin = *p;
        //     std::vector<int8_t> d = {1,1,1,100};
        //     grid->data = d;
        //     pubMap.publish(*grid); 
        // }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_visualize");
    ros::Time::init();
    ros::Rate rate(1);
    ASlamVisualize a;
    
    while(ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }
}