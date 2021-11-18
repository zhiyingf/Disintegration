# CMakeList.txt : CMake project for Disintegration ,include source and define
# project specific logic here.3.1
#
cmake_minimum_required (VERSION 3.1)

project (Disintegration)

# Add source to this project's executable.
file(GLOB SRCFILES src/*.cpp)
file(GLOB HEADERSFILES include/*.h)

include_directories(include/ src/ libigl/ libigl/eigen/)

add_executable(${PROJECT_NAME} ${HEADERSFILES} ${SRCFILES})


