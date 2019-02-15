#!/usr/bin/python

import argparse
import glob
import json
import os
import shutil
from subprocess import Popen, PIPE

FRC_PATH = os.path.expanduser('~/frc2019')
DESTINATION = FRC_PATH + '/extracted'


def create_dir_if_required(path):
    if not os.path.isdir(path):
        os.mkdir(path)


def create_dirs(do_clean=False):
    if do_clean and os.path.isdir(DESTINATION):
        shutil.rmtree(DESTINATION)
    create_dir_if_required(DESTINATION)
    create_dir_if_required(DESTINATION + '/include')
    create_dir_if_required(DESTINATION + '/src')
    create_dir_if_required(DESTINATION + '/libx86')
    create_dir_if_required(DESTINATION + '/libathena')


def extract(path, pkg, version=None):

    # Get the latest version
    if version is None:
        version = sorted(os.listdir(path + '/' + pkg))[-1]
        fullpath = '/'.join([path, pkg, version, pkg + '-' + version])
    else:
        fullpath = '/'.join([path, pkg + '-' + version])

    pkg = pkg.split('-')[0]
    print('\nExtracting {pkg} version {version}...'.format(pkg=pkg, version=version))

    # Extract headers
    file = fullpath + '-headers.zip'
    if (os.path.isfile(file)):
        Popen('unzip -uoq {0} -d {1}/include/{2}'.format(file, DESTINATION, pkg), shell=True).wait()

    # Extract sources
    file = fullpath + '-sources.zip'
    if (os.path.isfile(file)):
        Popen('unzip -uoq {0} -d {1}/src/{2}'.format(file, DESTINATION, pkg), shell=True).wait()

    # Extract native libraries
    for file in glob.glob(fullpath + '-linuxx86-64*.zip'):
        Popen('unzip -uoqj {0} linux/x86-64/* -d {1}/libx86/{2}'.format(file, DESTINATION, pkg), shell=True).wait()

    # Extract roborio libraries
    for file in glob.glob(fullpath + '-linuxathena*.zip'):
        Popen('unzip -uoqj {0} linux/athena/* -d {1}/libathena/{2}'.format(file, DESTINATION, pkg), shell=True).wait()

    print('Done extracting {pkg}'.format(pkg=pkg))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Extract maven artifacts. \
            By default, previously extracted files will be retained and freshened from the archives as required')
    parser.add_argument('-c', '--clean', action='store_true',
                        help='Clean existing extracted archives before extracting')
    args = parser.parse_args()

    # Create the required directories
    create_dirs(args.clean)

    # Extract WPILib artifacts
    wpi_maven_prefix = FRC_PATH + '/maven/edu/wpi/first'
    extract(wpi_maven_prefix + '/cameraserver', 'cameraserver-cpp')
    extract(wpi_maven_prefix + '/cscore', 'cscore-cpp')
    extract(wpi_maven_prefix + '/hal', 'hal-cpp')
    extract(wpi_maven_prefix + '/ntcore', 'ntcore-cpp')
    extract(wpi_maven_prefix + '/wpilibc', 'wpilibc-cpp')
    extract(wpi_maven_prefix + '/wpiutil', 'wpiutil-cpp')
    extract(wpi_maven_prefix + '/ni-libraries', 'chipobject')
    extract(wpi_maven_prefix + '/ni-libraries', 'netcomm')
    # extract(wpi_maven_prefix + '/thirdparty/frc2019', 'googletest')
    # extract(wpi_maven_prefix + '/thirdparty/frc2019/opencv', 'opencv-cpp')

    # Extract installed vendor artifacts
    vendor_deps = FRC_PATH + '/vendordeps'
    if os.path.isdir(vendor_deps):
        for file in os.listdir(vendor_deps):
            parsed = json.load(open(vendor_deps + '/' + file))

            for artifact in parsed['cppDependencies']:
                path = '/'.join([FRC_PATH, 'maven', artifact['groupId']]).replace('.', '/')
                extract(path, artifact['artifactId'], artifact['version'])
