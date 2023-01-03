cmake_minimum_required(VERSION 3.10)

project(NAME MXC400 
        LANGUAGES C 
        DESCRIPTION "A Hardware agnostic library for the MXC400 accelerometer")

file(GLOB SOURCES "src/*.c")

add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC include)