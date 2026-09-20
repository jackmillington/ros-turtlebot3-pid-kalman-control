cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_pid_kalman_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  message_generation
  roscpp
  std_msgs
)

add_message_files(
  FILES
  Sonars.msg
)

add_service_files(
  FILES
  ModelState.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    message_runtime
    roscpp
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sonars src/sonars.cc)
target_link_libraries(sonars ${catkin_LIBRARIES})
add_dependencies(sonars ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(noisy_sonars src/sonars.cc src/noise.cc)
target_compile_definitions(noisy_sonars PRIVATE NOISY_SONAR)
target_link_libraries(noisy_sonars ${catkin_LIBRARIES})
add_dependencies(noisy_sonars ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(model_state src/model_state.cc src/noise.cc)
target_link_libraries(model_state ${catkin_LIBRARIES})
add_dependencies(model_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sonar_variance_estimator src/sonar_variance_estimator.cpp)
target_link_libraries(sonar_variance_estimator ${catkin_LIBRARIES})
add_dependencies(sonar_variance_estimator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
