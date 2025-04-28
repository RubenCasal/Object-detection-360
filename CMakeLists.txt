cmake_minimum_required(VERSION 3.8)
project(tracker_360)

add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)

# === DEPENDENCIAS ===
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)  # ✅ necesario

# External dependencies via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# === INCLUDE DIRECTORIES ===
include_directories(
  include
  src
  ${GST_INCLUDE_DIRS}
  ${GST_BASE_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}  # ✅ después de find_package
  /usr/local/include
)

# === LINK DIRECTORIES ===
link_directories(
  ${GST_LIBRARY_DIRS}
  ${GST_BASE_LIBRARY_DIRS}
  ${GST_APP_LIBRARY_DIRS}
  ${GLIB_LIBRARY_DIRS}
  ${LIBUSB_LIBRARY_DIRS}
  /usr/local/lib
)

# === C++ SHARED LIB ===
add_library(tracker_360 SHARED
  src/ricoh_theta_node.cpp
  src/thetauvc.c
)

target_link_libraries(tracker_360
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${GST_BASE_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${GLIB_LIBRARIES}
  ${LIBUSB_LIBRARIES}
  uvc
  cv_bridge::cv_bridge  # ✅ linking explícito
)

ament_target_dependencies(tracker_360
  rclcpp
  sensor_msgs
  rclcpp_components
)

# === COMPONENT REGISTRATION ===
rclcpp_components_register_nodes(tracker_360 "theta_driver::ThetaDriver")

# === C++ EXECUTABLE ===
add_executable(theta_node src/theta_node.cpp)
target_link_libraries(theta_node tracker_360)
ament_target_dependencies(theta_node rclcpp sensor_msgs rclcpp_components cv_bridge)

# === INSTALL LIBS AND BINARIES ===
install(TARGETS
  tracker_360
  theta_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)
install(FILES
  yolov11n-face.pt
  DESTINATION share/${PROJECT_NAME}
)

# === PYTHON CONFIGURACIÓN ===
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/detector_node.py
  scripts/yolo_tracker.py
  scripts/multi_person_tracker.py
  scripts/crop_and_wrap.py
  scripts/stereo.py
  scripts/draw_boxes.py
  scripts/panoramic_projections.py
  DESTINATION lib/${PROJECT_NAME}  
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
