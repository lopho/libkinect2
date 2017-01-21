set(freenect2_DIR "" CACHE PATH "path to freenect2")

find_package(freenect2 REQUIRED)
include_directories(SYSTEM ${freenect2_INCLUDE_DIRS})

FIND_LIBRARY(kinect2_LIBRARY kinect2
    PATHS /home/zweistecken/Dokumente/Development/projection_mapping/workspace/libkinect2/install/lib
    NO_DEFAULT_PATH
)
SET(kinect2_LIBRARIES "${kinect2_LIBRARY};${freenect2_LIBRARIES}")

FIND_PATH(kinect2_INCLUDE_DIR kinect2/Kinect2.h
    PATHS /home/zweistecken/Dokumente/Development/projection_mapping/workspace/libkinect2/install/include
    NO_DEFAULT_PATH
)
SET(kinect2_INCLUDE_DIRS "${kinect2_INCLUDE_DIR};${freenect2_INCLUDE_DIRS}")

SET(kinect2_VERSION 0.1.0)
