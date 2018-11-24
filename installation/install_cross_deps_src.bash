#!/bin/bash
#
# Script used to install dependencies needed to cross-compile ROS.
# Whenever possible, we prefer to install IPKs that NI already distributes.
# However, when these aren't available, we instead build the dependencies from source.
#
# This script handles the dependencies that must be built from source.
#
# As much as possible, libraries are built as static to make it easier to install on the RIO
#

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
make_args="-S"

##### Launch Options #####

while [ $# -gt 0 ];
do
    case "$1" in
        -h|--help)
            echo "Options:"
            echo "-t FILE, --toolchain=FILE   = Specify the location of the toolchain file. If unspecified, default location used (Relative to script)"
            echo "-j [N], --jobs[=N]          = Specify the number of jobs to use when compiling with make. If unspecified, 4 jobs"
            echo "-q, --quiet                 = Don't echo make recepies"
            exit 0
            ;;

        # Toolchain file
        -t)
            shift
            toolchain="$1"
            shift
            ;;
        --toolchain*)
            toolchain=`echo $1 | sed -e 's/^[^=]*=//g'`
            shift
            ;;

        # Number of jobs
        -j)
            shift
            num_jobs="$1"
            shift
            ;;
        --jobs*)
            num_jobs=`echo $1 | sed -e 's/^[^=]*=//g'`
            shift
            ;;
        
        # Verbosity
        -mq)
            ;&
        --make_quiet)
            make_args+=" --quiet"
            shift
            ;;

        # Unknown option
        *)
            echo "Unknown option ${1}"
            exit 1
            ;;
    esac
done

##### Main script #####

# If the user specified a number of jobs, ensure it is a valid integer
if [ -z "$num_jobs" ]; then
    num_jobs=4
fi
if ! [ -z "$num_jobs" ]; then
    if ! [[ "$num_jobs" =~ ^[0-9]+$ ]] || [ $num_jobs -lt 1 ]; then
    echo "Error: $num_jobs is not a positive integer" >&2
    exit 1
    fi
fi

# Ensure the user-specified toolchain file. If they didn't provide one, use the default (relative to this script)
if [ -z "$toolchain" ]; then
    toolchain="${script_dir}/../rostoolchain.cmake"
fi
if [ ! -f "$toolchain" ]; then
    echo "Toolchain file "$toolchain" not found!"
    exit 1
fi

# Ensure the user is running as root
if (( $EUID != 0 )); then
    echo "Please run as root"
    exit 1
fi

# Used whenever a standard CMake compile is possible
# $1 is the name of the library, $2 is the url
function install_cmake ()
{
    archive="${1}_${2##*/}" # Prepend the archive name with the name of the library
	wget -q --show-progress $2 -O $archive
    mkdir -p ${1}/build
	tar xzf $archive --strip 1 -C $1
    cd ${1}/build
    cmake -DCMAKE_TOOLCHAIN_FILE="$toolchain" -DCMAKE_INSTALL_PREFIX=/usr/arm-frc-linux-gnueabi -DBUILD_SHARED_LIBS=OFF -DBUILD_STATIC_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release ..
    make install $make_args -j $num_jobs
    cd ../..
}

# Create temp directory to work in
rm -rf ~/.frc_control_temp
mkdir ~/.frc_control_temp
cd ~/.frc_control_temp

# class_loader v0.3.9 (Mar 16 2018) requires console_bridge. Corresponding console_bridge version for this date: v0.4.0
# tf2 v0.5.19 (Nov 17 2018) requires console_bridge. However, **tf2 requires v0.3.2**
install_cmake console_bridge https://github.com/ros/console_bridge/archive/0.3.2.tar.gz

# pluginlib v1.11.3 (Mar 16 2018) requires TinyXML2. Corresponding TinyXML2 version for this date: v6.0.0
install_cmake TinyXML2 https://github.com/leethomason/tinyxml2/archive/6.0.0.tar.gz

# TODO: urdfdom and urdfdom_headers can be installed either manually like this, or by merging them into the ros workspace and cloning their package.xml files. Not sure which is better
# urdf_parser_plugin v1.12.12 (Nov 8 2017) requires urdfdom. Corresponding version v1.0.0
install_cmake urdfdom https://github.com/ros/urdfdom/archive/1.0.0.tar.gz

# urdf_parser_plugin v1.12.12 (Nov 8 2017) requires urdfdom_headers. Corresponding version v1.0.0
install_cmake urdfdom_headers https://github.com/ros/urdfdom_headers/archive/1.0.0.tar.gz

# geometric_shapes v0.5.4 (May 14 2018) requires qhull. Corresponding version v2015.2
install_cmake qhull http://www.qhull.org/download/qhull-2015-src-7.2.0.tgz

# geometric_shapes v0.5.4 (May 14 2018) requires assimp. Corresponding version v4.1.0
install_cmake assimp https://github.com/assimp/assimp/archive/v4.1.0.tar.gz

# tf v1.11.9 (Jul 14 2017) requires gtest. Corresponding version v1.8.0
install_cmake googletest https://github.com/google/googletest/archive/release-1.8.0.tar.gz

# collada_parser v1.12.12 (May 8 2018) requires collada-dom. Corresponding version v2.5.0
install_cmake colllada-dom https://github.com/rdiankov/collada-dom/archive/v2.5.0.tar.gz

# class_loader v0.3.9 (Mar 16 2018) requires Poco. Corresponding Poco version for this date: v1.9.0
wget -q --show-progress https://github.com/pocoproject/poco/archive/poco-1.9.0-release.tar.gz
tar xzf poco-1.9.0-release.tar.gz
cd poco-poco-1.9.0-release
CROSS_COMPILE=arm-frc-linux-gnueabi- ./configure --no-tests --no-samples --minimal --prefix=/usr/arm-frc-linux-gnueabi/usr/local --static --cflags="-fPIC"
CROSS_COMPILE=arm-frc-linux-gnueabi- make install $make_args -j $num_jobs --quiet #We get TONS of warnings here if we don't compile quietly

# python_orocos_kdl v1.3.1 (Apr 5 2016) requires SIP. Corresponding SIP version for this date: v4.17
wget -q --show-progress https://sourceforge.net/projects/pyqt/files/sip/sip-4.17/sip-4.17.tar.gz
tar xzf sip-4.17.tar.gz
cd sip-4.17
python configure.py CC=arm-frc-linux-gnueabi-gcc CXX=arm-frc-linux-gnueabi-g++ LINK=arm-frc-linux-gnueabi-g++ LINK_SHLIB=arm-frc-linux-gnueabi-g++ --sysroot=/usr/arm-frc-linux-gnueabi --incdir=/usr/arm-frc-linux-gnueabi/usr/include/python2.7 STRIP=arm-frc-linux-gnueabi-strip
sed -i '/^CPPFLAGS/ s_include_usr/include_' siplib/Makefile
make install $make_args -j $num_jobs

# rospack v2.4.4 (Nov 16 2017) requires TinyXML. Corresponding TinyXML version for this date: v2.6.2
wget -q --show-progress https://sourceforge.net/projects/tinyxml/files/tinyxml/2.6.2/tinyxml_2_6_2.tar.gz
tar xzf tinyxml_2_6_2.tar.gz
cd tinyxml
# TinyXML's makefile is for gnu make. We need to create a CMakeLists. An existing template exists, so we will pull that
wget -q https://gist.githubusercontent.com/TNick/7960323/raw/3046ecda1d4d54d777c407f43ac357846a192e05/TinyXML-CmakeLists.txt -O CMakeLists.txt
# Force installation of the header file
sed -i "14i  set_target_properties(tinyxml PROPERTIES PUBLIC_HEADER tinyxml.h)" CMakeLists.txt
# Since we're installing the header, we also need to ensure the header and the library match. We will override the ENABLE_STL behaviour
# by forcing STL to be enabled in the header. This ensures that both the library and anyone using the header will have STL enabled
sed -i "29i#ifndef TIXML_USE_STL\n\t#define TIXML_USE_STL\n#endif\n" tinyxml.h
cmake -DCMAKE_TOOLCHAIN_FILE="$toolchain" -DCMAKE_INSTALL_PREFIX=/usr/arm-frc-linux-gnueabi -DBUILD_STATIC_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release .
make install $make_args -j $num_jobs

# bondcpp v1.8.3 (Aug 17 2018) requires uuid. Corresponding version v2.32.1
# Note: There might be a way to do with NI's ipks using ossp-libuuid. Not sure, haven't tried
wget -q --show-progress https://mirrors.edge.kernel.org/pub/linux/utils/util-linux/v2.32/util-linux-2.32.1.tar.gz
tar xzf util-linux-2.32.1.tar.gz
cd util-linux-2.32.1
./configure --host=arm-frc-linux-gnueabi --prefix=/usr/arm-frc-linux-gnueabi/usr/local --disable-all-programs --enable-libuuid
make install $make_args -j $num_jobs

# Cleanup
cd
rm -rf .frc_control_temp
