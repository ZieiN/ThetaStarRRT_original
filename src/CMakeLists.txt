cmake_minimum_required(VERSION 2.8)
project(ThetaStarRRT)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra")
set(SOURCES
        carlikerobot.cpp
        rrt.cpp
        posq.cpp
        thetastar.cpp
        map.cpp
        node.cpp
        position.cpp
        state.cpp
        robotstate.cpp
        geometricfunctions.cpp
        readconfigfunctions.cpp
        thetastarrrt.cpp
        ddrobot.cpp
        )
set(HEADERS
        carlikerobot.h
        robot.h
        rrt.h
        posq.h
        state.h
        robotstate.h
        map.h
        node.h
        thetastar.h
        position.h
        geometricfunctions.h
        readconfigfunctions.h
        thetastarrrt.h
        ddrobot.h
        )
add_executable(ThetaStarRRT ${HEADERS} ${SOURCES} main.cpp)