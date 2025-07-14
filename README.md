# swerve-drive-kinematics

## Usage
`cmake -B build`
`cmake --build build`
`./build/swerve` or `./build/tests`


## Profiling

- Compile with `-pg`
- run `gprof ./build/swerve gmon.out > analysis.txt`

## For future ref (cmake)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -Ofast")