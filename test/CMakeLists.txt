cmake_minimum_required(VERSION 3.5)
project("xdma_test")

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_LINUX "build on linux platform" ON)

if(BUILD_LINUX)
        add_definitions(-D_UNIX)
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

add_definitions(-DQT_NO_VERSION_TAGGING)

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -O0 -Wall -g -ggdb -std=c11 -fPIC")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -O3 -Wall -std=c11 -fPIC")

message("CMAKE_C_FLAGS_DEBUG is ${CMAKE_C_FLAGS_DEBUG}")
message("CMAKE_C_FLAGS_RELEASE is ${CMAKE_C_FLAGS_RELEASE}")
message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# option(SHOW_IMAGE "show image" ON)

# if(SHOW_IMAGE)
#         add_definitions(-D_SHOW_IMAGE)
# endif()

if(BUILD_LINUX)
	set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../3rd/opencv4.5/lib/cmake")
else()
	set(CMAKE_PREFIX_PATH "C:/3rd/opencv4.5/build/x64/vc15/lib")
endif()

find_package(OpenCV REQUIRED)
if(${OpenCV_FOUND})
        message(STATUS "Found Opencv" ${OpenCV_VERSION})
        include_directories(${OpenCV_INCLUDE_DIRS})
        link_libraries(${OpenCV_LIBS})
else()
        message(STATUS "Couldn't find Opencv")
endif()

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/../)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/../)

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/../)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/../)

include_directories(../src)
if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
        add_executable(${PROJECT_NAME}d main.cpp)
        target_link_libraries(${PROJECT_NAME}d fpgacontrol_xdmad pthread)
        message(STATUS "Debug Model")
else()
        add_executable(${PROJECT_NAME} main.cpp)
        target_link_libraries(${PROJECT_NAME} fpgacontrol_xdma pthread)
        message(STATUS "Release Model")
endif()
