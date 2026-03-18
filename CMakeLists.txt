cmake_minimum_required(VERSION 3.11)
project(fcat
    VERSION 0.6.1
    LANGUAGES C CXX
    DESCRIPTION "ROS2 Fastcat Node"
    )

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
endif()

# Dependencies
include(FetchContent)
FetchContent_Declare(fastcat
    GIT_REPOSITORY https://github.com/nasa-jpl/fastcat.git
    GIT_TAG v0.13.13
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
add_executable(fcat_node
    src/fcat_main.cpp
    src/fcat.cpp
    src/fcat_callbacks.cpp
    src/fcat_utils.cpp
)

ament_target_dependencies(fcat_node 
    rclcpp
    sensor_msgs
    geometry_msgs
    std_srvs
    fcat_msgs
    )
target_link_libraries(fcat_node
    fastcat
    )

# Create fcat_services node
add_executable(fcat_services
  src/fcat_srvs_main.cpp 
  src/fcat_srvs.cpp
  src/fcat_utils.cpp
  )
ament_target_dependencies(fcat_services 
  rclcpp 
  fcat_msgs
  )

target_link_libraries(fcat_services
  fastcat
  )

install(TARGETS
    fcat_node
    fcat_services
    DESTINATION lib/${PROJECT_NAME}
    )

ament_package()
