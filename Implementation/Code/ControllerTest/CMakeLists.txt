cmake_minimum_required(VERSION 3.0)
project(ControllerTest)

# set c++ version
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

# add executable for main.cpp
add_executable(ControllerTest main.cpp motor.cpp)

# include pigpio & pthread library
target_link_libraries(ControllerTest pigpio)
target_link_libraries(ControllerTest pthread)
target_link_libraries(ControllerTest rt)
