add_executable(shell-app main.cpp
 InverseKinematics.cpp
 StraightLinePath.cpp
 Coordinate.cpp
 Joints.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
