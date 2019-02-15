#!/bin/bash
#
# Script used to install dependencies needed to cross-compile ROS.
# Whenever possible, we prefer to install IPKs that NI already distributes.
# However, when these aren't available, we instead build the dependencies from source.
#
# This script handles the dependencies that can be installed from NI's IPKs

function install_package ()
{
    mkdir ~/.frc_control_temp
    cd ~/.frc_control_temp
    wget -q --show-progress "$1"
    ar x "${1##*/}" # Note: This regex simply extracts the filename from the url
    tar xzf data.tar.gz -C ~/frc2019/roborio/arm-frc2019-linux-gnueabi
    cd
    rm -rf .frc_control_temp
}

prefix="http://download.ni.com/ni-linux-rt/feeds/2018.5/arm/cortexa9-vfpv3/"

# Install Boost
install_package "${prefix}boost-dev_1.63.0-r1.5_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-atomic1.63.0_1.63.0-r1.5_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-chrono1.63.0_1.63.0-r1.5_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-date-time1.63.0_1.63.0-r1.5_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-filesystem1.63.0_1.63.0-r1.5_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-program-options1.63.0_1.63.0-r1.5_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-regex1.63.0_1.63.0-r1.5_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-signals1.63.0_1.63.0-r1.5_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-system1.63.0_1.63.0-r1.5_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-thread1.63.0_1.63.0-r1.5_cortexa9-vfpv3.ipk"

# Install Python
install_package "${prefix}python-core_2.7.13-r1.40_cortexa9-vfpv3.ipk"
install_package "${prefix}libpython2_2.7.13-r1.40_cortexa9-vfpv3.ipk"
install_package "${prefix}python-dev_2.7.13-r1.40_cortexa9-vfpv3.ipk"

# Install Eigen (Required for various packages (eigen, orocos, kdl, etc))
install_package "${prefix}libeigen_3.2.8-r0.5_cortexa9-vfpv3.ipk"

# Install FindEigen3.cmake
# Replaces deprecated FindEigen.cmake provided by the ROS cmake_modules package.
# TODO: Check this
# Note that we then need to manually specify the CMAKE_MODULE_PATH to include ~/frc2019/roborio/arm-frc-linux-gnueabi/usr/share/cmake/Modules
# See https://github.com/ros-perception/perception_pcl/issues/106
# For some reason, frcmake (cmake 3.5.1) doesn't have ~/frc2019/roborio/arm-frc-linux-gnueabi/usr/share/cmake/Modules in its modules path by default
install_package "${prefix}libeigen-dev_3.2.8-r0.5_cortexa9-vfpv3.ipk"

# Install cURL (Required for resource_retriever)
install_package "${prefix}libcurl4_7.53.1-r0.16_cortexa9-vfpv3.ipk"
install_package "${prefix}curl-dev_7.53.1-r0.16_cortexa9-vfpv3.ipk"

# Install lz4 (Required for roslz4)
install_package "${prefix}lz4_131+git0+d86dc91677-r0.19_cortexa9-vfpv3.ipk"
install_package "${prefix}lz4-dev_131+git0+d86dc91677-r0.19_cortexa9-vfpv3.ipk"

# Install BZip2 (Required for rosbag_storage)
install_package "${prefix}libbz2-1_1.0.6-r5.509_cortexa9-vfpv3.ipk"
install_package "${prefix}bzip2-dev_1.0.6-r5.509_cortexa9-vfpv3.ipk"

# Install UUID (Required for bondcpp)
# Unfortunately, only the library and the binary are included, not dev headers. Perhaps we should look into libossp-uuid later.
# For now, we just compile from source instead
#install_package "${prefix}libuuid1_2.29.1-r0.3.55_cortexa9-vfpv3.ipk"

# Install gpgme and its dependencies gpg-error and assuan (Required by rosbag_storage)
install_package "${prefix}gpgme-dev_1.8.0-r0.5_cortexa9-vfpv3.ipk"
install_package "${prefix}gpgme_1.8.0-r0.5_cortexa9-vfpv3.ipk"
install_package "${prefix}libgpg-error-dev_1.26-r1.28_cortexa9-vfpv3.ipk"
install_package "${prefix}libgpg-error0_1.26-r1.28_cortexa9-vfpv3.ipk"
install_package "${prefix}libassuan0_2.4.3-r0.5_cortexa9-vfpv3.ipk"

# Install openssl (Required by rosbag_storage)
install_package "${prefix}openssl-dev_1.0.2k-r0.32_cortexa9-vfpv3.ipk"

# Install libcrypto (Required by rosbag_storage)
install_package "${prefix}libcrypto1.0.2_1.0.2k-r0.32_cortexa9-vfpv3.ipk"
