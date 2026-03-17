cmake_minimum_required(VERSION 3.10)
project(indooruav_waypoint_manager)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── 依赖包 ────────────────────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

# ── catkin 导出 ───────────────────────────────────────────────────────────────
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nav_msgs
)

# ── 头文件目录 ────────────────────────────────────────────────────────────────
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ── 可执行目标 ────────────────────────────────────────────────────────────────
add_executable(indooruav_waypoint_manager_node
  src/main.cpp
  src/waypoint_manager.cpp
  src/recorder.cpp
)

target_link_libraries(indooruav_waypoint_manager_node
  ${catkin_LIBRARIES}
)

# ── 安装规则 ──────────────────────────────────────────────────────────────────
install(TARGETS indooruav_waypoint_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
