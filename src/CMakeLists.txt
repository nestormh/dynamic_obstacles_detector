add_library(dynamic_obstacles_detector
    DynamicObstaclesDetectorNodelet.cpp
)

include_directories(dynamic_obstacles_detector
    ${catkin_INCLUDE_DIRS}
    include
)

target_link_libraries(dynamic_obstacles_detector
  ${catkin_LIBRARIES}
)