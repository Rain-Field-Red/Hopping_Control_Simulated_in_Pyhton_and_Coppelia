cmake_minimum_required(VERSION 2.8.12)
project(simulation)

add_subdirectory(pybind11)
pybind11_add_module(simBridge src/simBridge.cpp)


# # Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
# # Copyright (C) 2006-2010 RobotCub Consortium
# # All rights reserved.
# #
# # This software may be modified and distributed under the terms of the
# # BSD-3-Clause license. See the accompanying LICENSE file for details.
# # YARP needs CMake 3.0 or greater
# cmake_minimum_required(VERSION 3.5)
# project(hello)
# # find YARP
# # find_package(YARP COMPONENTS os REQUIRED)
# find_package(YARP REQUIRED)
# # set up our program
# add_executable(hello)
# # declare our source files
# target_sources(hello PRIVATE hello.cpp)
# # link with YARP libraries
# # target_link_libraries(hello PRIVATE YARP::YARP_os
#                                     # YARP::YARP_init)
# target_link_libraries(hello ${YARP_LIBRARIES})