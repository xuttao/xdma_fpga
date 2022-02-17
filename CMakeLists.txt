cmake_minimum_required(VERSION 3.5)
project("fpgacontrol_xdma")

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_LINUX "build on linux platform" ON)
option(BUILD_2V7 "build 2v7 platform" ON)

if(BUILD_2V7)
	add_definitions(-D_DOUBLE_V7)
	set(yoloDetection src/detect/yoloDetect2.cpp)
	set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../3rd/opencv4.5/lib/cmake")
	message(STATUS ${CMAKE_PREFIX_PATH})
	find_package(OpenCV REQUIRED)
	if(${OpenCV_FOUND})
			message(STATUS "Found Opencv" ${OpenCV_VERSION})
			include_directories(${OpenCV_INCLUDE_DIRS})
			link_libraries(${OpenCV_LIBS})
	else()
			message(STATUS "Couldn't find Opencv")
	endif()
else()
set(yoloDetection src/detect/yoloDetect.cpp)
endif()

if(BUILD_LINUX)
    add_definitions(-D_UNIX)
else()
    add_definitions(-D_CONSOLE)
	add_definitions(-DXDMAFPGA_EXPORTS)
endif()

message(STATUS "Current CXX standard is ${CMAKE_CXX_STANDARD}")

# message("CMAKE_C_FLAGS_DEBUG is ${CMAKE_C_FLAGS_DEBUG}")
# message("CMAKE_C_FLAGS_RELEASE is ${CMAKE_C_FLAGS_RELEASE}")
# message("CMAKE_C_FLAGS_RELWITHDEBINFO is ${CMAKE_C_FLAGS_RELWITHDEBINFO}")
# message("CMAKE_C_FLAGS_MINSIZEREL is ${CMAKE_C_FLAGS_MINSIZEREL}")

# message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
# message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
# message("CMAKE_CXX_FLAGS_RELWITHDEBINFO is ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
# message("CMAKE_CXX_FLAGS_MINSIZEREL is ${CMAKE_CXX_FLAGS_MINSIZEREL}")

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb -std=c++17 -fPIC")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -std=c++17 -fPIC")


set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -O0 -Wall -g -ggdb -std=c11 -fPIC")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -O3 -Wall -std=c11 -fPIC")

message("CMAKE_C_FLAGS_DEBUG is ${CMAKE_C_FLAGS_DEBUG}")
message("CMAKE_C_FLAGS_RELEASE is ${CMAKE_C_FLAGS_RELEASE}")
message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(BUILD_LINUX)
	include_directories(src/xdma_linux)
	aux_source_directory(src/xdma_linux xdma_list)
else()
	include_directories(src/xdma_win)
	aux_source_directory(src/xdma_win xdma_list)
endif()

include_directories(src)
include_directories(src/detect)
aux_source_directory(src src_list)

if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
    add_library(${PROJECT_NAME}d SHARED ${src_list} ${xdma_list} ${yoloDetection})
    target_link_libraries(${PROJECT_NAME}d)
    message(STATUS "Debug Model")
else()
    add_library(${PROJECT_NAME} SHARED ${src_list} ${xdma_list} ${yoloDetection})
    target_link_libraries(${PROJECT_NAME})
    message(STATUS "Release Model")
endif()

#if(NOT BUILD_2V7)
#	add_subdirectory(test)
#endif()

add_subdirectory(test)
