cmake_minimum_required(VERSION 3.5)
project(kuka_kr6_support)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY config meshes urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 