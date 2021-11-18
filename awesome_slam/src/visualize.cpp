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

#include <ros/ros.h>
#include <awesome_slam_msgs/Landmarks.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace aslam {
class Visualize {
public:
    Visualize():nh(ros::NodeHandle()) {
        pubMarker   = nh.advertise<visualization_msgs::MarkerArray>("out/marker", 100);
        subLandmark = nh.subscribe("out/landmarks", 100, &aslam::Visualize::cbLandmarks, this);
    }
    
private:
    ros::NodeHandle nh;
    ros::Publisher pubMarker;
    ros::Subscriber subLandmark;
    

    void cbLandmarks(const awesome_slam_msgs::LandmarksConstPtr& msg) {
        int id = -1;
        float originalX[] = {3.0, -3.0, 0.0};
        float originalY[] = {0.0, 0.0, 3.0};
        visualization_msgs::MarkerArray markerArray;

        for (int i=0; i<msg->x.size(); i++) {
            visualization_msgs::Marker _markerP = createMarker(msg->x[i], msg->y[i], ++id, 
                                                                0.0f, 1.0f, 0.0f, ros::Duration(1/4.0));  
            visualization_msgs::Marker _markerO = createMarker(originalX[i], originalY[i], ++id, 
                                                                0.0f, 0.0f, 1.0f, ros::Duration());  
            markerArray.markers.push_back(_markerP);
            markerArray.markers.push_back(_markerO);
        }
        pubMarker.publish(markerArray);
    }


    visualization_msgs::Marker createMarker(double x, double y, int id, 
                                            float r, float g, float b, 
                                            const ros::Duration& duration) const {
        visualization_msgs::Marker marker;
        marker.id = id;
        marker.lifetime = duration;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        return marker;
    }
};
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "slam_visualize");
    ros::Time::init();
    ros::Rate rate(1);
    aslam::Visualize a;
    
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}