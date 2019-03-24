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
Script used to install common vendor libraries

Note: Unfortunately, most vendors seem to be misusing the 'sharedLibrary' and 'binaryPlatforms' tags.
We will have to manually sort through the available options
"""

from __future__ import print_function

import argparse
import json
import os
import subprocess

# Support Py2 and Py3:
try:
    import urllib.request as urllib2
except ImportError:
    import urllib2

# TODO: Port to frc2019? Bit late for this year, but it will hopefully let it be supported in the future?
DEPRECATED = {'mindsensors': 'http://mindsensors.com/largefiles/FIRST/mindsensors.zip'}

LIBRARIES = {
    'ADIS16448': 'http://maven.highcurrent.io/vendordeps/ADIS16448.json',
    'ADIS16470': 'http://maven.highcurrent.io/vendordeps/ADIS16470.json',
    'ctre': 'http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json',
    'digilent': 'http://s3-us-west-2.amazonaws.com/digilent/Software/DMC60C/DMC60C.json',
    'kauai': 'http://www.kauailabs.com/dist/frc/2019/navx_frc.json',
    'rev': 'http://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json'
}

COLORS = {'white': '37', 'red': '31', 'green': '32', 'yellow': '33', 'magenta': '35'}


def log(msg, color=COLORS['white'], bold=False):
    """
    Print with the specified color and formatting
    """
    print('\033[{color}{bold}m{msg}\033[0m'.format(color=color, msg=msg, bold=';1' if bold else ''))


def download(url, directory):
    """
    Download the specified URL to the specified directory, returning the downloaded file's full path
    """
    directory = os.path.expanduser(directory)
    subprocess.check_call(['wget', '-q', '--show-progress', url, '-N', '-P', directory])
    return directory + '/' + url.split("/")[-1]


def add_cmake_flag(flag, workspace):
    """
    Add the CMake flag to the catkin configuration on both native and cross profiles
    """
    subprocess.check_call(['catkin', 'config', '--profile', 'native', '--remove-args', flag], cwd=workspace)
    subprocess.check_call(['catkin', 'config', '--profile', 'cross', '--remove-args', flag], cwd=workspace)
    subprocess.check_call(['catkin', 'config', '--profile', 'native', '--append-args', flag], cwd=workspace)
    subprocess.check_call(['catkin', 'config', '--profile', 'cross', '--append-args', flag], cwd=workspace)


def main():
    """
    Main function
    """

    # pylint: disable=too-many-branches
    # pylint: disable=too-many-statements

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-c', '--catkin_ws', default=os.getcwd(), help='The workspace in which to enable the vendor library')
    parser.add_argument(
        '-d', '--download', action='store_true', help='Only download & install libraries, do not enable them')
    parser.add_argument('-a', '--all', action='store_true', help='Install all available libraries')
    for library in LIBRARIES:
        parser.add_argument(
            '--{0}'.format(library), action='store_true', help='Install the {0} library'.format(library))

    args = parser.parse_args()

    # Check if there exists a valid catkin workspace
    try:
        args.catkin_ws = subprocess.check_output(['catkin', 'locate'], cwd=args.catkin_ws).decode('utf-8').strip()
    except subprocess.CalledProcessError:
        if args.download:
            log('\nDirectory {dir} is not a part of any catkin workspace!'.format(dir=args.catkin_ws), COLORS['red'])
            log('Please specify a valid workspace\n', COLORS['red'])
            exit(1)

    if args.download:
        log('\nInstalling and enabling specified libraries in workspace {dir}'.format(dir=args.catkin_ws))
    else:
        log('\nDownloading specified libraries')

    # Iterate through every known library
    url_opener = urllib2.build_opener()
    url_opener.addheaders = [('User-Agent', 'Mozilla/5.0')]
    num_libs_success = 0
    num_libs_attempt = 0
    for library, json_url in LIBRARIES.items():
        if not args.all and not getattr(args, library):
            continue
        log('\nPreparing to install {0}...'.format(library), COLORS['magenta'], True)
        num_libs_attempt += 1
        try:

            # Download the JSON
            filename = download(json_url, "~/frc2019/vendordeps")
            parsed = json.loads(open(filename).read())
            log('Fetched JSON! Package: {0} - Version: {1}'.format(parsed['name'], parsed['version']))

            maven_url = parsed['mavenUrls'][0]

            # Download each cpp dependency
            log('Downloading artifacts...')
            for dep in parsed['cppDependencies']:
                url = '/'.join(
                    [maven_url.rstrip('/'), dep['groupId'].replace('.', '/'), dep['artifactId'], dep['version']])

                # Note: This is sloppy but at least it will cover the artifacts we need.
                # We will just allow wget to fail on nonexistant suffixes
                for suffix in ['headers', 'linuxathena', 'linuxathenastatic', 'linuxx86-64', 'llinuxx86-64static']:
                    name = '-'.join([dep['artifactId'], dep['version'], suffix])
                    try:
                        download(url + '/' + name + '.zip', '~/frc2019/maven/' + dep['groupId'].replace('.', '/'))
                    except subprocess.CalledProcessError:
                        pass
            log('Finished downloading')

            # Add -D<library>=ON when compiling
            if not args.download:
                flag = '-D{0}'.format(library.upper())
                log('Enabling {0} in frc_control...'.format(flag))
                add_cmake_flag(flag + '=ON', args.catkin_ws)
                log('Successfully enabled {0}'.format(flag))

            log('Finished installing {0} v{1}'.format(parsed['name'], parsed['version']), COLORS['green'])
            num_libs_success += 1
        except KeyboardInterrupt:
            log('Keyboard interrupt, aborting...', COLORS['yellow'])
            break
        except:  # pylint: disable=bare-except
            log('Failed to install {0}!'.format(library), COLORS['red'])

    log('')
    if num_libs_attempt == 0:
        log('No libraries specified for install!\n', COLORS['red'], True)
        exit(1)
    elif num_libs_success != num_libs_attempt:
        log('{0}/{1} libraries installed sucessfully\n'.format(num_libs_success, num_libs_attempt), COLORS['yellow'],
            True)
        exit(1)
    else:
        log('All specified libraries installed successfully\n', COLORS['green'], True)
        exit(0)


if __name__ == '__main__':
    main()
