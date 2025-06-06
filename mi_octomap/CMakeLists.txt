cmake_minimum_required(VERSION 3.8)
project(mi_octomap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
# new
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)

find_package(moveit_msgs REQUIRED)

# Declarar el ejecutable a partir de src/shape_detector_node.cpp
add_executable(shape_detector_node src/shape_detector_node.cpp)
# Agregar las dependencias (además de rclcpp y std_msgs, sensor_msgs es necesario para manejar PointCloud2)
ament_target_dependencies(
  shape_detector_node
  rclcpp
  std_msgs
  sensor_msgs
)
# Incluir las rutas de PCL y enlazar las bibliotecas
target_include_directories(shape_detector_node PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(shape_detector_node ${PCL_LIBRARIES})
include_directories(${pcl_conversions_INCLUDE_DIRS})

# Instalar el ejecutable
install(TARGETS
  shape_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(cube_intersection src/cube_intersection.cpp)
ament_target_dependencies(cube_intersection
  rclcpp
  octomap_msgs
  octomap
  sensor_msgs
  moveit_msgs
  shape_msgs
)
target_include_directories(cube_intersection PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(cube_intersection ${PCL_LIBRARIES})
include_directories(${pcl_conversions_INCLUDE_DIRS})

install(TARGETS cube_intersection
  DESTINATION lib/${PROJECT_NAME}
)

# --- Instalar el script Python como nodo ---
install(
  PROGRAMS
  src/paralelepipedo_interseccion.py
  src/triggers_set.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
