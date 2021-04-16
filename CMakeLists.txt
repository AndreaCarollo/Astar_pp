cmake_minimum_required(VERSION 3.5)

# Source path:
set(SOURCE_DIR ${CMAKE_HOME_DIRECTORY}/src)
# List of all files in src with name ending in '.cpp':
file(GLOB LIB_SOURCES ${SOURCE_DIR}/lib/*.cpp)

# TARGETS
# static library containing everything in src
#     first argument is the name of the target
#     second argument is the type of the library (STATIC of SHARED)
#     third argument is a list of source files
add_library(followmelib STATIC ${LIB_SOURCES})
target_link_libraries(followmelib m)

set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_STANDARD_REQUIRED TRUE )
SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -ltbb -O3")


project( Demo_PP )

find_package( OpenCV REQUIRED )
find_package( realsense2 REQUIRED )
find_package( InferenceEngine REQUIRED)
find_package( PCL 1.8 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${PROJECT_SOURCE_DIR} )
include_directories( ${InferenceEngine_INCLUDE_DIRS} )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions(  ${PCL_DEFINITIONS} )
include_directories( "/opt/intel/openvino_2020.3.194/inference_engine/external/tbb/include")
link_directories(    "/opt/intel/openvino_2020.3.194/inference_engine/external/tbb/lib")

add_executable( Demo_PP ${SOURCE_DIR}/main.cpp )
target_link_libraries( Demo_PP followmelib)
target_link_libraries( Demo_PP ${OpenCV_LIBS} )
target_link_libraries( Demo_PP ${realsense2_LIBRARY} )
target_link_libraries( Demo_PP ${InferenceEngine_LIBRARIES} tbb)
target_link_libraries( Demo_PP ${PCL_LIBRARIES} )


add_executable( Demo_PP_load ${SOURCE_DIR}/main_load.cpp )
target_link_libraries( Demo_PP_load followmelib)
target_link_libraries( Demo_PP_load ${OpenCV_LIBS} )
target_link_libraries( Demo_PP_load ${realsense2_LIBRARY} )
target_link_libraries( Demo_PP_load ${InferenceEngine_LIBRARIES} tbb)
target_link_libraries( Demo_PP_load ${PCL_LIBRARIES} )
