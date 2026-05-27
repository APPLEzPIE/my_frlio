cmake_minimum_required(VERSION 3.4.1)
project(frlio)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug") 

# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_CXX_STANDARD 17)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

# message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
# if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
#   include(ProcessorCount)
#   ProcessorCount(N)
#   message("Processer number:  ${N}")
#   if(N GREATER 4)
#     add_definitions(-DMP_EN)
#     add_definitions(-DMP_PROC_NUM=3)
#     message("core for MP: 3")
#   elseif(N GREATER 3)
#     add_definitions(-DMP_EN)
#     add_definitions(-DMP_PROC_NUM=2)
#     message("core for MP: 2")
#   else()
#     add_definitions(-DMP_PROC_NUM=1)
#   endif()
# else()
#   add_definitions(-DMP_PROC_NUM=1)
# endif()

# find_package(OpenMP QUIET)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
find_package(PythonLibs REQUIRED)


# add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")



list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake)

find_package(Eigen3 REQUIRED)
find_package(gflags REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})


include(cmake/ros.cmake)


include_directories(
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${yaml-cpp_INCLUDE_DIRS}
	include
)

# add_definitions(-DMY_DEBUG_MODE)
add_definitions(-DMY_VIS_MODE)
add_definitions(-DGEN_RGB_MAP)
# add_definitions(-DMY_TIME_DEBUG_MODE)


add_executable(frlio_ros_online ./src/frlio_ros_online.cpp)
target_link_libraries(frlio_ros_online 
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
	gflags
	glog
	yaml-cpp
)

add_executable(frlio_ros_offline_evaluate ./src/frlio_ros_offline_evaluate.cpp)
target_link_libraries(frlio_ros_offline_evaluate 
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES}
	gflags
	glog
	yaml-cpp
)