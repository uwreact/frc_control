#!/bin/bash
#
# Script used to install dependencies needed to cross-compile ROS.
# Whenever possible, we prefer to install IPKs that NI already distributes.
# However, when these aren't available, we instead build the dependencies from source.
#
# This script handles the dependencies that can be installed from NI's IPKs

## IMPORTANT NOTE: The 2018 frc_toolchain uses GCC 5.5.
# Due to ABI compatibility, this means we can only use libraries compiled with
# 5.5 or earlier. This means we CANNOT use the 2018 ipks from NI, since they're built
# on GCC 6.3. However, the 2017 ipks are safe, as they are built on GCC 5.3.
# In general, we have to just ensure we're using a valid version of the ni-linux-rt
# libraries

if (( $EUID != 0 )); then
    echo "Please run as root"
    exit
fi

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

prefix="http://download.ni.com/ni-linux-rt/feeds/2017/arm/cortexa9-vfpv3/"

## Install Boost
install_package "${prefix}boost-dev_1.60.0-r0.3_cortexa9-vfpv3.ipk"
# install_package "${prefix}boost-log_1.60.0-r0.3_cortexa9-vfpv3.ipk"
# install_package "${prefix}boost-serialization_1.60.0-r0.3_cortexa9-vfpv3.ipk"
# install_package "${prefix}boost-staticdev_1.60.0-r0.3_cortexa9-vfpv3.ipk"
# install_package "${prefix}boost-test_1.60.0-r0.3_cortexa9-vfpv3.ipk"
# install_package "${prefix}boost_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-atomic1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-chrono1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-date-time1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-filesystem1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-graph1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-iostreams1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-program-options1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-random1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-regex1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-signals1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-system1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-thread1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}libboost-timer1.60.0_1.60.0-r0.3_cortexa9-vfpv3.ipk"

## Install Python
install_package "${prefix}python-core_2.7.11-r1.49_cortexa9-vfpv3.ipk"
install_package "${prefix}libpython2_2.7.11-r1.49_cortexa9-vfpv3.ipk" # Required for python_orocos_kdl
install_package "${prefix}python-dev_2.7.11-r1.49_cortexa9-vfpv3.ipk" # Required for python_orocos_kdl

## Install Eigen (Required for various packages (eigen, orocos, kdl, etc))
install_package "${prefix}libeigen_3.2.6-r0.5_cortexa9-vfpv3.ipk"

# Install FindEigen3.cmake
# Replaces deprecated FindEigen.cmake provided by the ROS cmake_modules package.
# Note that we then need to manually specify the CMAKE_MODULE_PATH to include /usr/arm-frc-linux-gnueabi/usr/share/cmake/Modules
# See https://github.com/ros-perception/perception_pcl/issues/106
# For some reason, frcmake (cmake 3.5.1) doesn't have /usr/arm-frc-linux-gnueabi/usr/share/cmake/Modules in its modules path by default
install_package "${prefix}libeigen-dev_3.2.6-r0.5_cortexa9-vfpv3.ipk"

# Install cURL (Required for resource_retriever)
install_package "${prefix}libcurl4_7.51.0-r0.3_cortexa9-vfpv3.ipk"
install_package "${prefix}curl-dev_7.51.0-r0.3_cortexa9-vfpv3.ipk"

# Install lz4 (Required for roslz4)
install_package "${prefix}lz4_131+git0+d86dc91677-r0.4_cortexa9-vfpv3.ipk"
install_package "${prefix}lz4-dev_131+git0+d86dc91677-r0.4_cortexa9-vfpv3.ipk"

# Install BZip2 (Required for rosbag_storage)
install_package "${prefix}libbz2-1_1.0.6-r5.326_cortexa9-vfpv3.ipk"
install_package "${prefix}bzip2-dev_1.0.6-r5.326_cortexa9-vfpv3.ipk"

# Install UUID (Required for bondcpp)
# Unfortunately, only the library and the binary are included, not dev headers. Perhaps we should look into libossp-uuid later.
# For now, we just compile from source instead
#install_package "${prefix}libuuid1_2.27.1-r0.3.61_cortexa9-vfpv3.ipk"

# Install ZLIB (Required for collada-dom)
# TODO: Double check this is required
install_package "${prefix}libz-dev_1.2.8-r0.329_cortexa9-vfpv3.ipk"
install_package "${prefix}libz1_1.2.8-r0.329_cortexa9-vfpv3.ipk"

# Install libxml2 (Required for collada-dom)
install_package "${prefix}libxml2-dev_2.9.4-r0.49_cortexa9-vfpv3.ipk"
install_package "${prefix}libxml2_2.9.4-r0.49_cortexa9-vfpv3.ipk"

# Install libgnutls (Required by collada_urdf)
install_package "${prefix}libgnutls30_3.4.9-r0.7_cortexa9-vfpv3.ipk"

# Install libidn (Required by collada_urdf)
install_package "${prefix}libidn11_1.32-r0.7_cortexa9-vfpv3.ipk"

# Install libnettle (Required by collada_urdf)
install_package "${prefix}nettle_3.2-r0.7_cortexa9-vfpv3.ipk"

# Install libgmp (Required by collada_urdf)
install_package "${prefix}libgmp10_6.1.0-r0.44_cortexa9-vfpv3.ipk"
