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

#include "visualize_landmark.h"

aslam::VisualizeLandmark::VisualizeLandmark() : nh(ros::NodeHandle())
{
        nh.getParam("landmarks/x", orig_x);
        nh.getParam("landmarks/y", orig_y);

        pub_marker = nh.advertise<visualization_msgs::MarkerArray>("out/marker", 10);
        sub_landmark = nh.subscribe("out/landmarks/kalman", 10, &aslam::VisualizeLandmark::callback, this);
}

void aslam::VisualizeLandmark::callback(const awesome_slam_msgs::LandmarksConstPtr &msg)
{
        uint32_t size;
        int32_t id = -1;

        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray marker_array;

        // Create marker for groud truth positions
        size = orig_x.size();
        for (int i = 0; i < size; ++i)
        {
                marker = createMarker(orig_x[i], orig_y[i], 0.0f, 1.0f, ++id);
                marker_array.markers.push_back(marker);
        }

        // Create marker for predicted landmark positions
        size = msg->x.size();
        for (int i = 0; i < size; ++i)
        {
                marker = createMarker(msg->x[i], msg->y[i], 1.0f, 0.0f, ++id);
                marker_array.markers.push_back(marker);
        }

        pub_marker.publish(marker_array);
}

visualization_msgs::Marker aslam::VisualizeLandmark::createMarker(const float &pos_x, const float &pos_y, const float &color_green,
                                                                  const float &color_blue, const uint32_t id) const
{
        visualization_msgs::Marker marker;
        marker.id = id;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 1.0;
        marker.color.r = 0.0f;
        marker.color.a = 1.0f;
        marker.color.g = color_green;
        marker.color.b = color_blue;
        marker.pose.position.x = pos_x;
        marker.pose.position.y = pos_y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.header.frame_id = "odom";
        marker.lifetime = ros::Duration();
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::CYLINDER;
        return marker;
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "aslam_visualize_landmark");
        ros::Time::init();
        ros::Rate rate(1);
        aslam::VisualizeLandmark a;
        std::cerr << "[VISUALIZE LANDMARK] Node started!\n";

        while (ros::ok())
        {
                ros::spinOnce();
                rate.sleep();
        }
}