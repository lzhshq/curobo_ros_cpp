cmake_minimum_required(VERSION 3.22)
project(curobo_planner_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPose.srv"
  "srv/AddObject.srv"
  "srv/RemoveObject.srv"
  "srv/ClearObjects.srv"
  "srv/UpdateObjectPose.srv"
  "srv/ListObjects.srv"
  DEPENDENCIES geometry_msgs sensor_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
