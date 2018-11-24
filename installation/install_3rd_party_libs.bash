#!/bin/bash
#
# Script used to install common 3rd party libraries (CTRE, Kauai, and Mindsensors)
#

##### Launch Options #####

while [ $# -gt 0 ];
do
    case "$1" in
        -h|--help)
            echo "Options:"
            echo "--ws=[DIR]        = The ROS workspace to install the libraries in"
            echo "--ctre            = Install the CTRE libraries"
            echo "--kauai           = Install the Kauai libraries"
            echo "--mindsensors     = Install the Mindsensors libraries"
            echo "--all             = Install all supported libraries"
            exit 0
            ;;

        # Workspace
        --ws=*)
            dir=`echo $1 | sed -e 's/^[^=]*=//g'`
            shift
            ;;

        # CTRE libraries
        --ctre)
            ctre=true
            shift
            ;;

        # Kauai libraries
        --kauai)
            kauai=true
            shift
            ;;
        
        # Mindsensors libraries
        --mindsensors)
            mindsensors=true
            shift
            ;;

        # All libraries
        --all)
            all=true
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


if [ ! -z "$dir" ]; then
    cd $dir
fi

if [[ $(catkin locate 2>/dev/null) == "" ]]; then
    echo "Please specify a valid ROS workspace"
    exit 1
fi

if [ ! $ctre ] && [ ! $kauai ] && [ ! $mindsensors ] && [ ! $all ] ; then
    echo "No libraries specified for installation!"
fi


# Create temp directory to work in
rm -rf .frc_control_temp
mkdir .frc_control_temp
cd .frc_control_temp

# Ensure the install directory exists
mkdir -p /home/mreynolds/wpilib/user/cpp/lib
mkdir -p /home/mreynolds/wpilib/user/cpp/include

if [ $ctre ] || [ $all ] ; then
    echo "Installing CTRE libraries..."

    # Download the library
    wget -q --show-progress ctr-electronics.com//downloads/lib/CTRE_Phoenix_FRCLibs_NON-WINDOWS_v5.8.1.0.zip
    unzip -q -u CTRE_Phoenix_FRCLibs_NON-WINDOWS_v5.8.1.0.zip cpp/lib/* -d ~/wpilib/user/
    unzip -q -u CTRE_Phoenix_FRCLibs_NON-WINDOWS_v5.8.1.0.zip cpp/include/* -d ~/wpilib/user/
    rm CTRE_Phoenix_FRCLibs_NON-WINDOWS_v5.8.1.0.zip

    # Configure build profile to enable the library
    {
        catkin config --profile native --remove-args -DCTRE=ON
        catkin config --profile cross  --remove-args -DCTRE=ON
        catkin config --profile native --append-args -DCTRE=ON
        catkin config --profile cross  --append-args -DCTRE=ON
    } &> /dev/null
    echo "Done"
fi

if [ $kauai ] || [ $all ] ; then
    echo "Installing Kauai libraries..."

    # Download the library
    wget -q --show-progress www.kauailabs.com/public_files/navx-mxp/navx-mxp-libs.zip
    unzip -q -j -u navx-mxp-libs.zip roborio/cpp/lib/* -d ~/wpilib/user/cpp/lib
    unzip -q -j -u navx-mxp-libs.zip roborio/cpp/include/* -d ~/wpilib/user/cpp/include/kauai
    rm navx-mxp-libs.zip

    # Configure build profile to enable the library
    {
        catkin config --profile native --remove-args -DNAVX=ON
        catkin config --profile cross  --remove-args -DNAVX=ON
        catkin config --profile native --append-args -DNAVX=ON
        catkin config --profile cross  --append-args -DNAVX=ON
    } &> /dev/null
    echo "Done"
fi

if [ $mindsensors ] || [ $all ] ; then
    echo "Installing Mindsensors libraries..."

    # Download the library
    wget -q --show-progress mindsensors.com/largefiles/FIRST/mindsensors.zip
    unzip -q -j -u mindsensors.zip cpp/lib/* -d ~/wpilib/user/cpp/lib
    unzip -q -j -u mindsensors.zip cpp/include/* -d ~/wpilib/user/cpp/include/mindsensors
    rm mindsensors.zip

    # Configure build profile to enable the library
    {
        catkin config --profile native --remove-args -DMINDSENSORS=ON
        catkin config --profile cross  --remove-args -DMINDSENSORS=ON
        catkin config --profile native --append-args -DMINDSENSORS=ON
        catkin config --profile cross  --append-args -DMINDSENSORS=ON
    } &> /dev/null
    echo "Done"
fi

cd ..
rm -rf .frc_control_temp