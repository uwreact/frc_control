cmake_minimum_required(VERSION 2.8.3)
set(RIO_PATH $ENV{HOME}/frc2019/roborio)
set(ARM_PREFIX arm-frc2019-linux-gnueabi)

set(CMAKE_SYSTEM_NAME Linux)

# Setup the compilers for a cross-compile
set(CMAKE_C_COMPILER ${RIO_PATH}/bin/${ARM_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${RIO_PATH}/bin/${ARM_PREFIX}-g++)

# Tell CMake where to find resources
set(CMAKE_FIND_ROOT_PATH ${RIO_PATH}/${ARM_PREFIX})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
