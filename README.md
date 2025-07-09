# swerve-drive-kinematics

## Usage
`cmake -B build`
`cmake --build build`
`./build/swerve` or `./build/tests`


## Profiling

- Compile with `-pg`
- run `gprof ./build/swerve gmon.out > analysis.txt`