cmake_minimum_required(VERSION 3.3)
project(ihmc_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-sign-conversion)
endif()

# Set up project properties
set(PROJECT_NAME ihmc_msgs)
set(PROJECT_DESCRIPTION "IHMC ROS messages")

# TODO: amend this license message
# Print initial message
message(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")
message(
  STATUS "Copyright (C) 2020-2023 Heriot-Watt University, University of Oxford")
message(STATUS "All rights reserved.")
message(STATUS "Released under the BSD 3-Clause License.")

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# Define the Python modules
set(${PROJECT_NAME}_PYBIND11_MODULE crocoddyl_ros)

add_definitions(-DROS2)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# Find ROS2 required packages
set(PACKAGE_DEPENDENCIES rclcpp realtime_tools std_msgs geometry_msgs whole_body_state_msgs
                         pybind11)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(DEPENDENCY IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()
# Define messages to be generated
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/CapturabilityBasedStatus.msg"
  "msg/RobotConfigurationData.msg"
  "msg/IMUPacket.msg"
  "msg/SpatialVectorMessage.msg"
  "msg/Control.msg"
  "msg/FeedbackGain.msg"
  "msg/SolverStatistics.msg"
  "msg/SolverTrajectory.msg"
  "msg/State.msg"
  "msg/TimeInterval.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  whole_body_state_msgs)
# Add warning definitions
if(APPLE AND ${pybind11_VERSION} VERSION_LESS "2.10.3")
  add_definitions(-DIHMC_MSG_DISABLE_PYBIND11_WARNINGS)
endif()
# Build the Python interface for the ROS publisher and subscriber
foreach(PYBIND11_MODULE ${${PROJECT_NAME}_PYBIND11_MODULE})
  pybind11_add_module(${PYBIND11_MODULE} MODULE src/${PYBIND11_MODULE}.cpp)
  target_include_directories(
    ${PYBIND11_MODULE}
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           ${EIGEN3_INCLUDE_DIR} $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PYBIND11_MODULE} PUBLIC ${PACKAGE_DEPENDENCIES})
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                                rosidl_typesupport_cpp)
  target_link_libraries(
    ${PYBIND11_MODULE} PUBLIC ${cpp_typesupport_target} ${Boost_LIBRARIES}
                              ${Boost_PYTHON_LIBRARY} pinocchio::pinocchio)
  set_target_properties(${PYBIND11_MODULE} PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                                      lib/${PROJECT_NAME})
  install(TARGETS ${PYBIND11_MODULE}
          LIBRARY DESTINATION ${PYTHON_INSTALL_DIR})
endforeach(PYBIND11_MODULE ${${PROJECT_NAME}_PYBIND11_MODULE})
# Install the include files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()

add_subdirectory(unittest)
