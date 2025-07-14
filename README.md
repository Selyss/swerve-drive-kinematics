# swerve-drive-kinematics

## Usage
`cmake -B build`
`cmake --build build`
`./build/swerve` or `./build/tests`


## Profiling
- Compile with `-pg`
- run `gprof ./build/swerve gmon.out > analysis.txt`

## Tests
`pio test -e native` for kinematics library testing

## For future ref (cmake)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -Ofast")