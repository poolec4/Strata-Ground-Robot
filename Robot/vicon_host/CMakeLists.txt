cmake_minimum_required (VERSION 3.1)
project(FDCL-UAV)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package (PkgConfig REQUIRED)
pkg_check_modules (GTK3 REQUIRED gtk+-3.0)

include_directories (${GTK3_INCLUDE_DIRS})
link_directories (${GTK3_LIBRARY_DIRS})
add_definitions (${GTK3_CFLAGS_OTHER})

set (CMAKE_CXX_STANDARD 11)


link_directories("/usr/local/lib/")
include_directories("/usr/local/include/")

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/libraries/vectornav/include)
include_directories(${PROJECT_SOURCE_DIR}/libraries/eigen)
include_directories(${GTK3_INCLUDE_DIRS})

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy) 

SET(EXECUTABLE_OUTPUT_PATH ".")

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

# set(CMAKE_CXX_FLAGS "-Wall -c -O3 -lvrpn -lquat -pthread -std=c++11 ${CMAKE_C_FLAGS}")


################################################################################
#### Make Host
################################################################################
set(base_src
    src/main.cpp
)
add_executable(base ${base_src})
target_link_libraries(base
    host
    vrpn
    Threads::Threads
    ${GTK3_LIBRARIES}
)
target_compile_options(base PRIVATE -pthread -Wall -o -lrt -lm -lvrpn -lquat -pthread -lserialport -lsbp -O3 -std=c++11)


################################################################################
#### Compile host libraries
################################################################################

set(host_src
    ${PROJECT_SOURCE_DIR}/src/fdcl_param.cpp
    ${PROJECT_SOURCE_DIR}/src/fdcl_vicon.cpp
    ${PROJECT_SOURCE_DIR}/src/fdcl_robot.cpp
    ${PROJECT_SOURCE_DIR}/src/misc_matrix_func.cpp

)
add_library(host STATIC ${host_src})
