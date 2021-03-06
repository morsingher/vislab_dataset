cmake_minimum_required(VERSION 3.5)
project(multi_view_clustering)

set(CMAKE_CXX_FLAGS "-O2 -march=native -mtune=native -mavx")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} src/include)

add_executable(multi_view_clustering 
			   src/include/input_dataset.cc 
			   src/include/math_utils.cc
			   src/include/clustering.cc
			   src/include/parameters.cc
			   src/main.cc)

target_link_libraries(multi_view_clustering ${OpenCV_LIBS} Threads::Threads)