cmake_minimum_required(VERSION 3.5)
set(USE_GL 1)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)

#Mujoco paths
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries)

#Finding main mujoco library
if(${USE_GL})
    file(GLOB LIB_MUJOCO ${CMAKE_CURRENT_SOURCE_DIR}/libraries/libmujoco[0-9][0-9][0-9].so)
else()
    file(GLOB LIB_MUJOCO ${CMAKE_CURRENT_SOURCE_DIR}/libraries/libmujoco[0-9][0-9][0-9]nogl.so)
endif()
#Showing mujoco library found
message(STATUS "MuJoCo lib: " ${LIB_MUJOCO})

#file(GLOB LIB_GLFW ${CMAKE_CURRENT_SOURCE_DIR}/libraries/libglfw.so)

find_package(OpenGL)
find_package(Eigen3)
find_package(OsqpEigen)#ADDED

file(GLOB LIB_GLFW3 ${CMAKE_CURRENT_SOURCE_DIR}/libraries/libglfw.so.3)


add_library(utils SHARED ${CMAKE_CURRENT_SOURCE_DIR}/include/uitools.c)

target_link_libraries(utils GL GLU glew ${LIB_MUJOCO} ${LIB_GLFW3})

add_library(eigenUtils SHARED eigen/eigenUtils.cpp)


add_executable(basic sample/basic.cpp)
add_executable(balancerWrench sample/balancerWrench.cpp)


add_executable(simulate sample/simulate.cpp)

# libraries to link against
target_link_libraries(basic ${LIB_MUJOCO} GL GLU glew ${LIB_GLFW3})
target_link_libraries(balancerWrench ${LIB_MUJOCO} GL GLU glew ${LIB_GLFW3} eigenUtils)

target_link_libraries(simulate ${LIB_MUJOCO} GL GLU glew ${LIB_GLFW3} utils)
target_link_libraries(balancerWrench OsqpEigen::OsqpEigen)#added
