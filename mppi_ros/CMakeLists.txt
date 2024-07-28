cmake_minimum_required(VERSION 3.8)
project(mppi_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# necessary to get past eigen resizing (?)
set(CMAKE_BUILD_TYPE RelWithDebInfo) #None, Debug, Release, RelWithDebInfo, MinSizeRel

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED 3.3)

# set(PACKAGE_DEPENDENCIES
#         # rosbag
#         mppi
#         sensor_msgs
#         std_msgs
#         rclcpp
#         # roscpp
#         # message_generation
#         rosidl_default_generators
# )
# find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(ament_cmake REQUIRED)
find_package(mppi REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(message_generation REQUIRED)
find_package(mppi_ros_interfaces REQUIRED)

# # # Messages
# # add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msg
# #         FILES Array.msg Config.msg Rollout.msg Data.msg)
# # generate_messages(DEPENDENCIES std_msgs)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Array.msg"
#   "msg/Config.msg"
#   "msg/Data.msg"
#   "msg/Rollout.msg"
#   DEPENDENCIES builtin_interfaces std_msgs
# )

# ament_export_dependencies(rosidl_default_runtime)

# catkin_package(
#         LIBRARIES ${PROJECT_NAME}
#         INCLUDE_DIRS include
#         CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} message_runtime)

# set(COMMON_LIBRARIES ${catkin_LIBRARIES})
# set(SYSTEM_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include_directories(
  include
  SYSTEM ${SYSTEM_INCLUDE_DIRECTORIES})

include_directories(
  ${mppi_INCLUDE_DIRS}
  )

# Libraries
add_library(mppi_threading
  src/threading/Rate.cpp
  src/threading/Worker.cpp
  src/threading/WorkerManager.cpp
)

add_library(${PROJECT_NAME}
  src/controller_interface.cpp
  src/conversions.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  mppi
  # ${PROJECT_NAME}_interfaces
  mppi_ros_interfaces
  # sensor_msgs
  # std_msgs
  # yaml-cpp
  # OpenMP
  rclcpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS} ${mppi_INCLUDE_DIRS})

# target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES} mppi_threading)
target_link_libraries(
  ${PROJECT_NAME}
  mppi_threading
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} mppi_threading)

# catkin_install_python(PROGRAMS scripts/plot_experiment.py scripts/record_data.py
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )
# ## Testing
# if (CATKIN_ENABLE_TESTING)
#     #    catkin_add_gtest()
#     #    target_link_libraries()

# endif ()

install(TARGETS
  ${TARGET}
  DESTINATION lib/${PROJECT_NAME})

ament_package()