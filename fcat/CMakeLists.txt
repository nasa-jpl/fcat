cmake_minimum_required(VERSION 3.22)
project(fcat LANGUAGES C CXX)

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

# Compile-time maximum log level for jsd/fastcat (levels above this are removed entirely)
# 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG
set(FCAT_MAX_LOG_LEVEL "3" CACHE STRING
  "Compile-time max log level for jsd/fastcat: 1=ERROR, 2=+WARN, 3=+INFO, 4=+DEBUG")

# Bounds jsd/fastcat ERROR/WARNING logged from the realtime process thread, where
# a bus fault reasserts every cycle. 0 disables throttling.
set(FCAT_LOG_THROTTLE_MS "1000" CACHE STRING
  "Throttle interval (ms) for realtime-thread jsd/fastcat ERROR/WARNING; 0 = no throttling")

# Needed before FetchContent below, which compiles against the rcutils headers.
find_package(rcutils REQUIRED)

# Dependencies
include(FetchContent)
FetchContent_Declare(fastcat
  GIT_REPOSITORY https://github.com/nasa-jpl/fastcat.git
  GIT_TAG v0.13.20
)
FetchContent_MakeAvailable(fastcat)

# Override jsd_print.h so jsd/fastcat log macros go through rcutils instead of
# fprintf(stderr). BEFORE makes src/jsd/jsd_print.h win over the copy in
# ${CMAKE_BINARY_DIR}/include that "jsd/jsd_print.h" normally resolves to.
#
# The two target_link_libraries() calls must keep the signature each target was
# first linked with upstream; mixing plain and keyword on one target is fatal.
target_include_directories(fastcat BEFORE PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_definitions(fastcat PRIVATE
  FCAT_MAX_LOG_LEVEL=${FCAT_MAX_LOG_LEVEL}
  FCAT_LOG_THROTTLE_MS=${FCAT_LOG_THROTTLE_MS}
)
target_link_libraries(fastcat rcutils::rcutils)  # plain signature, matching upstream
target_include_directories(jsd-lib BEFORE PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_definitions(jsd-lib PRIVATE
  FCAT_MAX_LOG_LEVEL=${FCAT_MAX_LOG_LEVEL}
  FCAT_LOG_THROTTLE_MS=${FCAT_LOG_THROTTLE_MS}
)
target_link_libraries(jsd-lib PRIVATE rcutils::rcutils)  # keyword signature, matching upstream
target_sources(jsd-lib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/fcat_log.c)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fcat_msgs REQUIRED)

find_program(CLANG_FORMAT_BIN NAMES clang-format)
if(CLANG_FORMAT_BIN)
  file(GLOB_RECURSE FCAT_FORMAT_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hh"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
  )

  add_custom_target(format
    COMMAND ${CLANG_FORMAT_BIN} -i -style=file ${FCAT_FORMAT_FILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Running clang-format on source files"
  )
else()
  message(STATUS "clang-format not found; 'format' target is unavailable for fcat")
endif()

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

target_include_directories(fcat_node BEFORE PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_definitions(fcat_node PRIVATE
  FCAT_MAX_LOG_LEVEL=${FCAT_MAX_LOG_LEVEL}
  FCAT_LOG_THROTTLE_MS=${FCAT_LOG_THROTTLE_MS}
)

target_link_libraries(fcat_node
  rclcpp::rclcpp
  rcutils::rcutils
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${fcat_msgs_TARGETS}
  fastcat
)

# Create fcat_services node
add_executable(fcat_services
  src/fcat_srvs_main.cpp
  src/fcat_srvs.cpp
  src/fcat_utils.cpp
)

target_include_directories(fcat_services BEFORE PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_definitions(fcat_services PRIVATE
  FCAT_MAX_LOG_LEVEL=${FCAT_MAX_LOG_LEVEL}
  FCAT_LOG_THROTTLE_MS=${FCAT_LOG_THROTTLE_MS}
)

target_link_libraries(fcat_services
  rclcpp::rclcpp
  rcutils::rcutils
  ${fcat_msgs_TARGETS}
  fastcat
)

install(TARGETS
  fcat_node
  fcat_services
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format QUIET)
  if(ament_cmake_clang_format_FOUND)
    ament_clang_format(
      CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format
    )
  endif()
endif()

ament_package()
