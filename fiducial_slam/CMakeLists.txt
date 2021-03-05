cmake_minimum_required(VERSION 2.8.3)
project(fiducial_slam)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
  cv_bridge
  sensor_msgs
  std_msgs
  fiducial_msgs
  genmsg
)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
find_package(G2O REQUIRED)
find_package(Eigen3 REQUIRED)

##############
## Services ##
##############

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AddFiducial.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS} include)
catkin_package(INCLUDE_DIRS include
  DEPENDS OpenCV)

include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}, ${G2O_INCLUDE_DIRS})

add_executable(fiducial_slam src/fiducial_slam.cpp 
src/Fiducial.cpp src/Observation.cpp
src/base_map.cpp src/map.cpp src/transform_with_variance.cpp)
add_dependencies(fiducial_slam ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_executable(g2o_fiducial_map src/g2o_fiducial_map.cpp
               src/base_map.cpp src/transform_with_variance.cpp
               src/Fiducial.cpp src/Observation.cpp
               )
add_dependencies(g2o_fiducial_map ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_executable(g2o_fiducial_slam src/g2o_fiducial_slam.cpp
                 src/base_map.cpp src/transform_with_variance.cpp
                 src/Fiducial.cpp src/Observation.cpp
                 )
add_dependencies(g2o_fiducial_slam ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
  

target_link_libraries(fiducial_slam ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(g2o_fiducial_map ${catkin_LIBRARIES} ${OpenCV_LIBS} ${G2O_LIBRARIES})
target_link_libraries(g2o_fiducial_slam ${catkin_LIBRARIES} ${OpenCV_LIBS} ${G2O_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS fiducial_slam
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES fiducials.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

###########
## Tests ##
###########

if(CATKIN_ENABLE_TESTING)
        find_package(catkin REQUIRED COMPONENTS
          rostest roscpp sensor_msgs tf2_ros image_transport cv_bridge)

        # Tests need c++11
        add_definitions(-std=c++11)
	
	catkin_add_gtest(transform_var_test test/transform_var_test.cpp src/transform_with_variance)
	target_link_libraries(transform_var_test ${catkin_LIBRARIES})

        add_rostest(test/create_map_aruco.xml)
        add_rostest(test/init_map_aruco.xml)

        add_rostest_gtest(auto_init_403_test
           test/auto_init_403.test
           test/auto_init_403_test.cpp)
        target_link_libraries(auto_init_403_test
            ${catkin_LIBRARIES}
            ${OpenCV_LIBS})

endif()
