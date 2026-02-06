cmake_minimum_required(VERSION 3.15)
project(biguit VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) 


find_package(PkgConfig REQUIRED)
find_package(SDL3 REQUIRED)
find_package(box2d REQUIRED)

# Add executable
add_executable(bigunit src/main.c src/data.c src/physicsdata.c)
# set_target_properties(bigunit PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Link libraries to the executable
target_link_libraries(bigunit PRIVATE SDL3::SDL3 box2d::box2d)
