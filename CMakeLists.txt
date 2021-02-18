cmake_minimum_required(VERSION 3.5)
project(vislab_dataset)

set(CMAKE_CXX_FLAGS "-O2 -march=native -mtune=native -mavx")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)
find_package(Threads REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} 
					${PYTHON_INCLUDE_DIRS}
					src/include)

add_executable(vislab_dataset 
			   src/include/input_dataset.cpp 
			   src/include/math_utils.cpp
			   src/include/view_clustering.cpp
			   src/include/parameters.cpp
			   src/main.cpp)

target_link_libraries(vislab_dataset ${OpenCV_LIBS} ${PYTHON_LIBRARIES} Threads::Threads)