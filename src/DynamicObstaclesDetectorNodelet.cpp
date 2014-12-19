/*
 *  Copyright 2014 Néstor Morales Hernández <nestor@isaatc.ull.es>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 * 
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
#include <pluginlib/class_list_macros.h>
#include "dynamic_obstacles_detector/DynamicObstaclesDetectorNodelet.h"

#include <sensor_msgs/image_encodings.h>

#include <string>

#include <iostream>
#include </opt/ros/indigo/include/ros/node_handle.h>

using namespace std;

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(dynamic_obstacles_detector, DynamicObstaclesDetectorNodelet, 
                        dynamic_obstacles_detector::DynamicObstaclesDetectorNodelet, nodelet::Nodelet) 

namespace dynamic_obstacles_detector
{
    
void DynamicObstaclesDetectorNodelet::onInit() 
{
    ros::NodeHandle& nh = getPrivateNodeHandle();
    string inputParam;
    const string DEFAULT_INPUT_PARAM = "input_point_cloud";
    nh.param("input", inputParam, DEFAULT_INPUT_PARAM);
    
    cout << "**********INPUT PARAM " << inputParam << endl;
    
    m_inputPointCloudSub = nh.subscribe(inputParam.c_str(), 1, &DynamicObstaclesDetectorNodelet::callback, this);
    
}
    
void DynamicObstaclesDetectorNodelet::callback(const sensor_msgs::PointCloud2::ConstPtr& msgPointCloud)
{
    
    PointCloudPtr pointCloud(new PointCloud());
    pcl::fromROSMsg<pcl::PointXYZRGB>(*msgPointCloud, *pointCloud);
    
    cout << __FILE__ << ":" << __LINE__ << " => Received point cloud with size " << pointCloud->size() << endl;
}
    
}