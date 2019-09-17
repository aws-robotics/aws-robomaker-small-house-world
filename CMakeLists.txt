cmake_minimum_required(VERSION 3.5)
project(aws_robomaker_small_house_world)

find_package(gazebo_ros REQUIRED)
find_package(ament_cmake_ros REQUIRED)
# find_package(turtlebot3_description REQUIRED)
# find_package(turtlebot3_navigation2 REQUIRED) # required for copy of .rviz file

install(DIRECTORY launch models worlds maps photos
    DESTINATION share/${PROJECT_NAME}
)

# Copy the rviz model for easier access in AWS RoboMaker RViz 
#install(FILES ${turtlebot3_navigation_DIR}/../rviz/turtlebot3_navigation.rviz
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
#  RENAME turtlebot3_navigation.rviz
#)

ament_export_dependencies(gazebo_ros)

ament_package()

