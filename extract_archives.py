#!/usr/bin/env python

##############################################################################
# Copyright (C) 2019, UW REACT
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of UW REACT, nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
##############################################################################

"""
Manually extract maven archives for use with CMake
"""

from __future__ import print_function

import argparse
import glob
import json
import os
import shutil
import subprocess

FRC_PATH = os.path.expanduser('~/frc2019')
DESTINATION = FRC_PATH + '/extracted'


def create_dir_if_required(path):
    """
    If it does not exist, create the specified directory
    """
    if not os.path.isdir(path):
        os.mkdir(path)


def create_destinatin_dirs(do_clean=False):
    """
    Create the destination directories
    """
    if do_clean and os.path.isdir(DESTINATION):
        shutil.rmtree(DESTINATION)
    create_dir_if_required(DESTINATION)
    create_dir_if_required(DESTINATION + '/include')
    create_dir_if_required(DESTINATION + '/src')
    create_dir_if_required(DESTINATION + '/libx86')
    create_dir_if_required(DESTINATION + '/libathena')


def extract(path, pkg, version=None):
    """
    Extract the specified Maven archives to the destination directory
    """
    # Get the latest version
    if version is None:
        version = sorted(os.listdir(path + '/' + pkg))[-1]
        fullpath = '/'.join([path, pkg, version, pkg + '-' + version])
    else:
        fullpath = '/'.join([path, pkg + '-' + version])

    pkg = pkg.split('-')[0]
    print('\nExtracting {pkg} version {version}...'.format(pkg=pkg, version=version))

    # Extract headers
    archive = fullpath + '-headers.zip'
    if os.path.isfile(archive):
        subprocess.check_call(['unzip', '-uoq', archive, '-d', '{0}/include/{1}'.format(DESTINATION, pkg)])

    # Extract sources
    archive = fullpath + '-sources.zip'
    if os.path.isfile(archive):
        subprocess.check_call(['unzip', '-uoq', archive, '-d', '{0}/src/{1}'.format(DESTINATION, pkg)])

    # Extract native libraries
    for archive in glob.glob(fullpath + '-linuxx86-64*.zip'):
        subprocess.check_call(
            ['unzip', '-uoqj', archive, 'linux/x86-64/*', '-d', '{0}/libx86/{1}'.format(DESTINATION, pkg)])

    # Extract roborio libraries
    for archive in glob.glob(fullpath + '-linuxathena*.zip'):
        subprocess.check_call(
            ['unzip', '-uoqj', archive, 'linux/athena/*', '-d', '{0}/libathena/{1}'.format(DESTINATION, pkg)])

    print('Done extracting {pkg}'.format(pkg=pkg))


def main():
    """
    Main function
    """
    parser = argparse.ArgumentParser(description='Extract maven artifacts. \
            By default, previously extracted files will be retained and freshened from the archives as required')
    parser.add_argument(
        '-c', '--clean', action='store_true', help='Clean existing extracted archives before extracting')
    args = parser.parse_args()

    # Create the required directories
    create_destinatin_dirs(args.clean)

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
        for json_file in os.listdir(vendor_deps):
            parsed = json.load(open(vendor_deps + '/' + json_file))

            for artifact in parsed['cppDependencies']:
                path = '/'.join([FRC_PATH, 'maven', artifact['groupId']]).replace('.', '/')
                extract(path, artifact['artifactId'], artifact['version'])


if __name__ == '__main__':
    main()
