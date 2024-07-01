# MIT License : Copyright (c) 2024 Yukiyoshi Sasao
cmake_minimum_required(VERSION 3.1.15)
project(OrbbecKinfu)

add_definitions( -std=c++11 )
add_definitions( -Wall )
add_definitions( -g )

add_executable(${PROJECT_NAME} main.cpp)

set(OrbbecSDK_DIR "/your/path/to/OrbbecSDK" CACHE STRING "path to OrbbecSDK")
find_package(OrbbecSDK REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
	OrbbecSDK::OrbbecSDK OrbbecSDK::DepthEngine
	${OpenCV_LIBRARIES}
)
