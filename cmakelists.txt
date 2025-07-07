cmake_minimum_required(VERSION 3.10)
project(swerve-drive-kinematics)

set(CMAKE_CXX_STANDARD 23)

set(SOURCES
    src/robot.cpp
    src/swerveModule.cpp
    main.cpp
)

include_directories(src)

add_executable(swerve ${SOURCES})