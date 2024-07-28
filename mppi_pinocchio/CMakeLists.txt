cmake_minimum_required(VERSION 3.8)
project(mppi_pinocchio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# necessary to get past eigen resizing (?)
set(CMAKE_BUILD_TYPE RelWithDebInfo) #None, Debug, Release, RelWithDebInfo, MinSizeRel

find_package(Eigen3 REQUIRED 3.3)
find_package(pinocchio REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)

# set(PACKAGE_DEPENDENCIES geometry_msgs)
# find_package(catkin REQUIRED ${PACKAGE_DEPENDENCIES})

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
#   DEPENDS pinocchio
# )

include_directories(
 include
 ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/model.cpp src/ros_conversions.cpp src/ros_conversions.cpp)
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} pinocchio::pinocchio)


ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  pinocchio
)


install(TARGETS
  ${TARGET}
  DESTINATION lib/${PROJECT_NAME})

# install(
#     DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME}
# )

# install(
#     DIRECTORY res
#     DESTINATION share/${PROJECT_NAME}
# )

ament_package()