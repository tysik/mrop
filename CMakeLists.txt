cmake_minimum_required(VERSION 2.8.3)
project(mrop)

set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS tf joy roscpp std_msgs std_srvs geometry_msgs)
#find_package(obstacle_detector REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Boost 1.46.1 REQUIRED system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS tf joy roscpp std_msgs std_srvs geometry_msgs
  DEPENDS system_lib)

include_directories(include ${catkin_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})# ${obstacle_detector_INCLUDE_DIRS})

#add_executable(mtracker src/mtracker.cpp include/mtracker.h include/serial.h)
#target_link_libraries(mtracker ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(mtracker ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(simulator src/simulator.cpp include/simulator.h)
target_link_libraries(simulator ${catkin_LIBRARIES})

add_executable(reference_generator src/reference_generator.cpp include/reference_generator.h include/trajectories.h)
target_link_libraries(reference_generator ${catkin_LIBRARIES})

add_executable(manual_controller src/manual_controller.cpp include/manual_controller.h)
target_link_libraries(manual_controller ${catkin_LIBRARIES})

#add_executable(automatic_controller src/automatic_controller.cpp include/automatic_controller.h)
#target_link_libraries(automatic_controller ${catkin_LIBRARIES})
#add_dependencies(automatic_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(obstacle_controller src/obstacle_controller.cpp include/obstacle_controller.h)
#target_link_libraries(obstacle_controller ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
#add_dependencies(obstacle_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(controls_scaling src/controls_scaling.cpp include/controls_scaling.h)
#target_link_libraries(controls_scaling ${catkin_LIBRARIES})
#add_dependencies(controls_scaling ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(state_estimator src/state_estimator.cpp include/state_estimator.h)
#target_link_libraries(state_estimator ${catkin_LIBRARIES})
#add_dependencies(state_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(data_recorder src/data_recorder.cpp include/data_recorder.h)
#target_link_libraries(data_recorder ${catkin_LIBRARIES})# libyaml-cpp.a)
#add_dependencies(data_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS})
