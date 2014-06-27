project(Paketvermessung)
cmake_minimum_required(VERSION 2.8)

set(OPENNI2_LIBRARY_DIRS /usr/local/lib/Redist)
set(OPENNI2_INCLUDE_DIRS /usr/local/include/Include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/local/include -L/usr/local/lib -std=c++0x")

find_package(PCL 1.7 REQUIRED common filters io visualization)


include_directories(${PCL_INCLUDE_DIRS} ${OPENNI2_INCLUDE_DIRS} /usr/local/include)
link_directories(${PCL_LIBRARY_DIRS} ${OPENNI2_LIBRARY_DIRS} /usr/local/lib)

set(grabber_HEADERS src/grabber.h)

set(EXECUT_SOURCES src/main.cpp src/grabber.cpp)

set(dir ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_BUILD_FILES_DIRECTORY ${dir}/build)
set(CMAKE_BUILD_DIRECTORY ${dir}/build)

add_subdirectory(src)

aux_source_directory(src/* SRC_LIST)

file(GLOB HEADERS src/*.h src/*.hpp)
file(GLOB SOURCES src/*.cpp)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
#add_library(${PROJECT_NAME}_lib SHARED ${HEADERS} ${SOURCES})

add_executable(${PROJECT_NAME} ${EXECUT_SOURCES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OPENNI2_LIBRARIES})

