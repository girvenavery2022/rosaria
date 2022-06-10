cmake_minimum_required(VERSION 3.5)
project(rosaria)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosaria_msgs REQUIRED)

find_package(Aria QUIET)
if(Aria_FOUND)
  message("CMake package for Aria was found, using that.")
  # Aria package for CMake was found
	if(EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
		add_definitions(-DADEPT_PKG)
	endif()
  include_directories(${Aria_INCLUDE_DIRS})
  link_directories(${Aria_LIBRARY_DIRS})
else()
  # No Aria package for CMake was found, search ourselves
	# The installation package provided by Adept doesn't follow normal Linux
	# installation locations, but the repackaged Debian package and AriaCoda do.
  # If user set ARIA environment variable, look there, otherwise look in
  # /usr/local/.
  if(DEFINED ENV{ARIA})
    message("ARIA environment variable defined, checking there...")
    set(prefix $ENV{ARIA})
  else()
    set(prefix "/usr/local")
  endif()
  message("Looking for Aria in ${prefix}/Aria and ${prefix}...")

	if(EXISTS "${prefix}/Aria/include/Aria.h")
    message("Found ${prefix}/Aria/include/Aria.h, assuming Adept ARIA package.")
	  add_definitions(-DADEPT_PKG)
	  include_directories(${prefix}/Aria/include)
	  link_directories(${prefix}/Aria/lib)
  else()
    if(EXISTS "${prefix}/include/Aria.h")
      message("Found ${prefix)/include/Aria.h, assuming Adept ARIA source directory.")
      add_definitions(-DADEPT_PKG)
      include_directories(${prefix}/include)
      link_directories(${prefix}/lib)
    else()
      if(EXISTS "${prefix}/include/Aria/Aria.h")
        message("Found ${prefix}/include/Aria/Aria.h, assuming AriaCoda or repackaged ARIA.")
        #add_definitions(-DARIACODA)
        include_directories(${prefix}/include)
        link_directories(${prefix}/lib)
      else()
        message("Aria.h not found in ${prefix}. Continuing with default header and library paths.")
      endif()
    endif()
	endif()
endif()

# BUILDING
add_executable(
  RosAria RosAria.cpp LaserPublisher.cpp
)
add_dependencies(RosAria rosaria_gencfg)
add_dependencies(RosAria rosaria_gencpp)

ament_target_dependencies(
  RosAria
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  rosaria_msgs
)

target_include_directories(RosAria PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
install(TARGETS RosAria
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()