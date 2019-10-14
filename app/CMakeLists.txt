add_executable(shell-app main.cpp
 InverseKinematics.cpp
 StraightLinePath.cpp
 Coordinate.cpp
 Joints.cpp
 Demo.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

find_package(PythonLibs 2.7)
target_include_directories( shell-app PRIVATE ${PYTHON_INCLUDE_DIR})
target_link_libraries( shell-app ${PYTHON_LIBRARY})
