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

# Tests

include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.8.1 # or a newer release
)

FetchContent_MakeAvailable(Catch2)

add_executable(tests test.cpp src/robot.cpp src/swerveModule.cpp)
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)