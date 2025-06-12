cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
# Agregado:
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# Para camara:
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
# Para control:
find_package(rclpy REQUIRED)
# Para point cloud
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
# Para guardar:
find_package(srdfdom REQUIRED)
# Para mejorar trayectoria
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)



add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})

# Ejecutable bye_moveit
add_executable(bye_moveit src/bye_moveit.cpp)
target_include_directories(bye_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bye_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  bye_moveit
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  geometry_msgs
  moveit_msgs
  shape_msgs
  Eigen3
)
install(TARGETS bye_moveit
  DESTINATION lib/${PROJECT_NAME})

# Ejecutable try_moveit
add_executable(try_moveit src/try_moveit.cpp)
target_include_directories(try_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(try_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  try_moveit
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  geometry_msgs
  moveit_msgs
  shape_msgs
  Eigen3
)
install(TARGETS try_moveit
  DESTINATION lib/${PROJECT_NAME})

# Ejecutable prueba_moveit
add_executable(prueba_moveit src/prueba_moveit.cpp)
target_include_directories(prueba_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(prueba_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  prueba_moveit
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  geometry_msgs
  moveit_msgs
  shape_msgs
  Eigen3
)
install(TARGETS prueba_moveit
  DESTINATION lib/${PROJECT_NAME})

# Ejecutable para camara detector:
# Declarar el ejecutable a partir de src/shape_detector_node.cpp
add_executable(detector_node src/detector_node.cpp)
# Agregar las dependencias (además de rclcpp y std_msgs, sensor_msgs es necesario para manejar PointCloud2)
ament_target_dependencies(
  detector_node
  rclcpp
  std_msgs
  sensor_msgs
  moveit_msgs
  shape_msgs
  pcl_conversions
  PCL
  moveit_ros_planning_interface
)
# Incluir las rutas de PCL y enlazar las bibliotecas
target_include_directories(detector_node PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(detector_node ${PCL_LIBRARIES})
include_directories(${pcl_conversions_INCLUDE_DIRS})

# Instalar el ejecutable
install(TARGETS
  detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Nodo Object Spawner
add_executable(invocador_node src/invocador_node.cpp)
ament_target_dependencies(invocador_node
  rclcpp
  moveit_msgs
  moveit_ros_planning_interface
)
# Si object_spawner_node usa funciones de PCL, puedes enlazar ${PCL_LIBRARIES} también.
target_link_libraries(invocador_node ${PCL_LIBRARIES})

# Instalar ejecutables
install(TARGETS
  invocador_node
  DESTINATION lib/${PROJECT_NAME}
)


# Nodo Dualsense.py
# Instala el ejecutable
install(PROGRAMS
  src/dual_sense_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)


# Nodo dualsense controller cpp
add_executable(dual_sense_teleop_node src/control_dual_sense.cpp)
ament_target_dependencies(dual_sense_teleop_node
  rclcpp
  moveit_msgs
  moveit_ros_planning_interface
  moveit_core
)
# Si dual_sense_teleop_node usa funciones de PCL, puedes enlazar ${PCL_LIBRARIES} también.
# target_link_libraries(dual_sense_teleop_node ${PCL_LIBRARIES})

# Instalar ejecutables
install(TARGETS
  dual_sense_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)


# Nodo world pointcloud cpp
add_executable(world_pointcloud src/world_pointcloud.cpp)
ament_target_dependencies(world_pointcloud
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
#  pcl_ros
)
target_include_directories(world_pointcloud PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(world_pointcloud ${PCL_LIBRARIES})
include_directories(${pcl_conversions_INCLUDE_DIRS})

# Instalar ejecutables
install(TARGETS
  world_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)


# Nodo guardar datos
add_executable(save_node src/save_node.cpp)
ament_target_dependencies(save_node
  rclcpp
  geometry_msgs
  std_msgs
  moveit_msgs
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  moveit_ros_planning_interface
  srdfdom
)

# Instalar ejecutables
install(TARGETS
  save_node
  DESTINATION lib/${PROJECT_NAME}
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
