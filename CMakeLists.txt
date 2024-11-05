cmake_minimum_required(VERSION 2.8.3)
project(fastlio_tum)

ADD_COMPILE_OPTIONS(-std=c++17 )

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosbag
        rospy
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        geometry_msgs
        nav_msgs
        tf
        message_generation
        eigen_conversions
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fastlio_tum
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
        # include
        # ${catkin_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)


link_directories(
        ${catkin_LIBRARY_DIRS}
        ${PCL_LIBRARY_DIRS}
)


#add_executable(bag_image_extra src/bag_extractor.cpp)
add_executable(tum_to_topic src/pubilsh_tum_to_odometry.cpp)
add_executable(publish_PC2 src/publish_pcl_cloud_2_PC2.cpp)
add_executable(publish_depth_PC2 src/publish_depth_2_PC2.cpp)
add_executable(publish_depth_PC2_2 src/publish_depth_2_PC2_2.cpp)
#target_link_libraries(bag_image_extra
#        ${catkin_LIBRARIES}
#        ${OpenCV_LIBRARIES}
#        ${PCL_LIBRARIES}
#)

target_link_libraries(tum_to_topic
        ${catkin_LIBRARIES}
)
target_link_libraries(publish_PC2
    ${catkin_LIBRARIES}
)
target_link_libraries(publish_depth_PC2
        ${catkin_LIBRARIES}
)
target_link_libraries(publish_depth_PC2_2
        ${catkin_LIBRARIES}
)

