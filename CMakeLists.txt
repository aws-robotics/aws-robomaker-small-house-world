cmake_minimum_required(VERSION 3.5)
project(aws_robomaker_small_house_world)

find_package(ament_cmake_ros REQUIRED)

install(DIRECTORY launch models worlds maps photos param routes
    DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/aws_robomaker_small_house_world.dsv.in")

ament_export_dependencies(gazebo_ros)

ament_package()

