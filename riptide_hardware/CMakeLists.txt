cmake_minimum_required(VERSION 2.8.3)
project(riptide_hardware)
add_compile_options(-std=c++11)

# NOTE: Most dependencies can be taken care of with $(catkin_EXPORTED_TARGETS).
# There is no need to add mention the specific dependency, such as
# <package_name>_gencpp, or <package_name>_gencfg

# NOTE: To include OpenCV, must do find_package(OpenCV REQUIRED), followed by
# include_directories(...)

find_package(catkin REQUIRED
    COMPONENTS
    roslint
    roscpp
    riptide_msgs
    geometry_msgs
    imu_3dm_gx4
    pointgrey_camera_driver
    message_filters
    tf
    message_generation
    sensor_msgs
    cv_bridge
)

add_service_files(FILES Pneumatics.srv)

generate_messages()

catkin_package(INCLUDE_DIRS include)
find_package(OpenCV REQUIRED) # To include, OpenCV, this MUST go BEFORE include_directories()
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} resources/acoustics/include)


roslint_cpp()

# include boost
find_package(Boost REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

cmake_host_system_information(RESULT _host_name QUERY HOSTNAME)
message("-- Computer name: " ${_host_name})

if(NOT _host_name MATCHES "jetson")

    add_executable(acoustics src/acoustics.cpp)
    target_link_libraries(acoustics ${catkin_LIBRARIES} okFrontPanel fftw3)
    add_dependencies(acoustics ${catkin_EXPORTED_TARGETS})

    add_executable(depth_processor src/depth_processor.cpp)
    target_link_libraries(depth_processor ${catkin_LIBRARIES})
    add_dependencies(depth_processor ${catkin_EXPORTED_TARGETS})
else()
    message("-- Skipping riptide code on jetson")
endif()

if(NOT _host_name MATCHES "riptide")

    add_executable(calibrate_camera src/calibrate_camera.cpp)
    target_link_libraries(calibrate_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
    add_dependencies(calibrate_camera ${catkin_EXPORTED_TARGETS})

    add_executable(undistort_camera src/undistort_camera.cpp)
    target_link_libraries(undistort_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
    add_dependencies(undistort_camera ${catkin_EXPORTED_TARGETS})
else()
    message("-- Skipping jetson code on riptide")
endif()

add_executable(imu_processor src/imu_processor.cpp)
target_link_libraries(imu_processor ${catkin_LIBRARIES})
add_dependencies(imu_processor ${catkin_EXPORTED_TARGETS})

add_executable(test_pub_sub src/test_pub_sub.cpp)
target_link_libraries(test_pub_sub ${catkin_LIBRARIES})
add_dependencies(test_pub_sub ${catkin_EXPORTED_TARGETS})

add_executable(test_sub_pub src/test_sub_pub.cpp)
target_link_libraries(test_sub_pub ${catkin_LIBRARIES})
add_dependencies(test_sub_pub ${catkin_EXPORTED_TARGETS})
