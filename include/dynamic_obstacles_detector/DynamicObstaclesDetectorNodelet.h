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

#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <omp.h>
#include <boost/thread/pthread/mutex.hpp>

#define INIT_CLOCK(start) double start = omp_get_wtime();
#define RESET_CLOCK(start) start = omp_get_wtime();
#define END_CLOCK(time, start) float time = omp_get_wtime() - start;
#define END_CLOCK_2(time, start) float time = omp_get_wtime() - start;

namespace dynamic_obstacles_detector {
    
    class DynamicObstaclesDetectorNodelet : public nodelet::Nodelet {
    public:
        void onInit();
        
    protected:
        typedef pcl::PointXYZRGB PointType;
        typedef pcl::PointCloud< PointType > PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        
        void callback(const sensor_msgs::PointCloud2::ConstPtr &msgPointCloud);

        ros::Subscriber m_inputPointCloudSub;
        
        //     boost::barrier m_barrier;
        boost::mutex m_mutex;
    };
    
}