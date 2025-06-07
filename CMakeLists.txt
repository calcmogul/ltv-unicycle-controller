cmake_minimum_required(VERSION 3.21)

# Set default build type to release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

# Generate compile_commands.json by default
if(NOT CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS "YES" CACHE STRING "" FORCE)
endif()

project(ltv-unicycle-controller)

file(GLOB_RECURSE cpp_src src/*.cpp)
set_property(SOURCE ${cpp_src} PROPERTY COMPILE_FLAGS "-Wno-unused")
add_executable(ltv-unicycle-controller ${cpp_src})

target_compile_options(ltv-unicycle-controller PRIVATE -Wall -Wextra -pedantic -Werror)
target_compile_features(ltv-unicycle-controller PUBLIC cxx_std_11)
target_include_directories(ltv-unicycle-controller PRIVATE src eigen3)
