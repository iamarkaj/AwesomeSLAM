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

#include "gazebo_spawn.h"

aslam::GazeboSpawn::GazeboSpawn() : nh(ros::NodeHandle()) {
  ros::service::waitForService("/gazebo/spawn_sdf_model");
  ros::ServiceClient client =
      nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");

  // Read positions from config/landmarks.yaml
  std::vector<double> origX;
  std::vector<double> origY;
  nh.getParam("landmarks/x", origX);
  nh.getParam("landmarks/y", origY);

  // Read model sdf
  std::string model_xml_path;
  nh.getParam("/gazebo_spawn/model_xml_path", model_xml_path);
  std::cout << model_xml_path << "\n";
  std::ifstream file(model_xml_path);
  std::string model_xml = {std::istreambuf_iterator<char>(file), {}};

  // Set common model desc
  gazebo_msgs::SpawnModel model;
  model.request.robot_namespace = "aslam";
  model.request.reference_frame = "world";
  model.request.model_xml = model_xml;

  // Set common pose desc
  geometry_msgs::Pose initial_pose;
  initial_pose.position.z = 0.0;
  initial_pose.orientation.x = 0.0;
  initial_pose.orientation.y = 0.0;
  initial_pose.orientation.z = 0.0;
  initial_pose.orientation.w = 0.0;

  // Spawn model
  uint32_t N = origX.size();
  for (uint32_t i = 0; i < N; ++i) {
    initial_pose.position.x = origX[i];
    initial_pose.position.y = origY[i];

    model.request.initial_pose = initial_pose;
    model.request.model_name = "cylinder_" + std::to_string(i);

    client.call(model);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "aslam_gazebo_spawn");
  aslam::GazeboSpawn a;

  std::cerr << "[GAZEBO SPAWN] Model spawned!\n";
}