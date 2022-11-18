PROJECT(amrl_msgs)
CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION is not defined")
endif()

if(${ROS_VERSION} EQUAL "1")
  message(STATUS "Compiling with ROS1 ROSBUILD")
  INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  ROSBUILD_INIT()
  SET(ROS_BUILD_STATIC_LIBS true)
  SET(ROS_BUILD_SHARED_LIBS false)

  ROSBUILD_GENMSG()
  ROSBUILD_GENSRV()
elseif(${ROS_VERSION} EQUAL "2")
  message(STATUS "Compiling with ROS2 COLCON")
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/AckermannCurvatureDriveMsg.msg"
    "msg/ColoredArc2D.msg"
    "msg/ColoredLine2D.msg"
    "msg/ColoredPoint2D.msg"
    "msg/ElevatorCommand.msg"
    "msg/ElevatorStatus.msg"
    "msg/ErrorReport.msg"
    "msg/HumanStateArrayMsg.msg"
    "msg/HumanStateMsg.msg"
    "msg/Localization2DMsg.msg"
    "msg/PathVisualization.msg"
    "msg/PlanarObjDetection.msg"
    "msg/Point2D.msg"
    "msg/Pose2Df.msg"
    "msg/RobofleetStatus.msg"
    "msg/RobofleetSubscription.msg"
    "msg/VisualizationMsg.msg"
    DEPENDENCIES geometry_msgs nav_msgs std_msgs
  )

  ament_package()
else()
  message(FATAL_ERROR "Unknown ROS_VERSION ${ROS_VERSION}")
endif()
