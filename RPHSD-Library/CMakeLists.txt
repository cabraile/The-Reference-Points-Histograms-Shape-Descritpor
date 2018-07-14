if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

# Project Defaults
set( INCLUDE_DIR "include" )
set( DEMO_DIR "demo" )
set( BUILD_DIR "build" )

# Compilation Standards
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# User Parameters
set( DEMO_NAME "experimental" )

# PCL Defaults
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(RPHSD)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Compilation
add_executable (${DEMO_NAME} ${DEMO_DIR}/${DEMO_NAME}.cpp)
target_link_libraries (${DEMO_NAME} ${PCL_LIBRARIES})
