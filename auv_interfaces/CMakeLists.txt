cmake_minimum_required(VERSION 3.8)
project(auv_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Actuator.msg"
  "msg/ArduinoSensor.msg"
  "msg/BoundingBox.msg"
  "msg/VelocitySensor.msg"
  "msg/Error.msg"
  "msg/Gerak.msg"
  "msg/Heading.msg"
  "msg/IsStable.msg"
  "msg/Logging.msg"
  "msg/Manual.msg"
  "msg/Movement.msg"
  "msg/MultiPID.msg"
  "msg/ObjectDetection.msg"
  "msg/ObjectDifference.msg"
  "msg/Orientation.msg"
  "msg/PID.msg"
  "msg/PIDVel.msg"
  "msg/Sensor.msg"
  "msg/SetPoint.msg"
  # DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
