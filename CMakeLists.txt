project(libkinect2)
set(APP kinect2)
set(EXAMPLE "${APP}_example")
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wpedantic")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11 -Wall -Wextra -Wpedantic")
if(${CMAKE_BUILD_TYPE} MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Og -g")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Og -g")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
endif()


set(freenect2_DIR "" CACHE PATH "path to libfreenect2")
find_package(freenect2 REQUIRED)
include_directories(SYSTEM ${freenect2_INCLUDE_DIRS})
include_directories("include")

file(GLOB_RECURSE SOURCES
    "src/*.c"
    "src/*.cpp"
    "include/*.h"
    "include/*.hpp"
)

file(GLOB_RECURSE EXAMPLE_SOURCES
    "example/*.c"
    "example/*.cpp"
    "example/*.h"
    "example/*.hpp"
)

add_library(
    ${APP}
    SHARED
    ${SOURCES}
)

add_executable(
    ${EXAMPLE}
    ${EXAMPLE_SOURCES}
)

target_link_libraries(
    ${APP}
    ${freenect2_LIBRARY}
)

target_link_libraries(
    ${EXAMPLE}
    ${APP}
)

set_target_properties(${APP} PROPERTIES OUTPUT_NAME ${APP})

if(CMAKE_BUILD_TYPE MATCHES Debug)
    #set_target_properties(${APP} PROPERTIES OUTPUT_NAME "${APP}d")
endif()

install(TARGETS ${APP} DESTINATION "lib")
install(DIRECTORY "include/kinect2" DESTINATION "include")
install(FILES "kinect2Config.cmake" DESTINATION ".")

