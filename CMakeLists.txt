cmake_minimum_required(VERSION 3.11)
project(fcat
    VERSION 0.5.1
    LANGUAGES C CXX
    DESCRIPTION "ROS2 Fastcat Node"
    )

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Werror)
endif()

# Dependencies
include(FetchContent)
FetchContent_Declare(fastcat
    GIT_REPOSITORY https://github.com/nasa-jpl/fastcat.git
    GIT_TAG v0.4.6
    )
FetchContent_MakeAvailable(fastcat)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fcat_msgs REQUIRED)


# Copies headers to install space 
file(GLOB_RECURSE includes RELATIVE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
    )
foreach(include ${includes})
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/${include}"
        "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}/${include}"
        COPYONLY
        )
endforeach()

# Create fcat node
add_executable(fcat
    src/fcat_node.cpp
    src/fcat.cpp
    src/fcat_callbacks.cpp
    )

ament_target_dependencies(fcat 
    rclcpp
    sensor_msgs
    geometry_msgs
    std_srvs
    fcat_msgs
    )
target_link_libraries(fcat
    fastcat
    )

# Create fcat_srvs node
add_executable(fcat_srvs
  src/fcat_srvs_node.cpp 
  src/fcat_srvs.cpp
  )
ament_target_dependencies(fcat_srvs 
  rclcpp 
  fcat_msgs
  )

target_link_libraries(fcat_srvs
  fastcat
  )

install(TARGETS
    fcat
    fcat_srvs
    DESTINATION lib/${PROJECT_NAME}
    )

ament_package()
