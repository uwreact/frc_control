cmake_minimum_required(VERSION 2.8.3)
set(ARM_PREFIX arm-frc-linux-gnueabi)

set(CMAKE_SYSTEM_NAME Linux)

# Setup the compilers for a cross-compile
set(CMAKE_C_COMPILER ${ARM_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${ARM_PREFIX}-g++)

# Tell CMake where to find resources
set(CMAKE_FIND_ROOT_PATH /usr/${ARM_PREFIX})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Ensure Boost uses the correct version
#TODO: Is this needed?
set(BOOST_ROOT ${ARM_PREFIX})
set(Boost_NO_SYSTEM_PATHS=ON)