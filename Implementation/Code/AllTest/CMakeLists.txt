cmake_minimum_required(VERSION 3.0)
project(AllTest)

# set c++ version
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

# add executable for main.cpp
add_executable(AllTest main.cpp motor.cpp PrunedNatNet.cpp)

# include pigpio & pthread library
target_link_libraries(AllTest pigpio)
target_link_libraries(AllTest pthread)
target_link_libraries(AllTest rt)
