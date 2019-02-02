#!/usr/bin/python
#
# Script used to install common vendor libraries
#
# Note: Unfortunately, most vendors seem to be misusing the 'sharedLibrary' and 'binaryPlatforms' tags.
# We will have to manually sort through the available options

import argparse
import json
import os
from subprocess import Popen, PIPE
import urllib2

# TODO: Port to frc2019? Bit late for this year, but it will hopefully let it be supported in the future?
deprecated = {'mindsensors': 'http://mindsensors.com/largefiles/FIRST/mindsensors.zip'}

libraries = {'ADIS16448': 'http://maven.highcurrent.io/vendordeps/ADIS16448.json',
             'ADIS16470': 'http://maven.highcurrent.io/vendordeps/ADIS16470.json',
             'ctre': 'http://devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json',
             'digilent': 'http://s3-us-west-2.amazonaws.com/digilent/Software/DMC60C/DMC60C.json',
             'kauai': 'http://www.kauailabs.com/dist/frc/2019/navx_frc.json',
             'rev': 'http://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json'}

colors = {'white': '37', 'red': '31', 'green': '32', 'yellow': '33', 'magenta': '35'}


def log(msg, color=colors['white'], bold=False):
    print('\033[{color}{bold}m{msg}\033[0m'.format(color=color, msg=msg, bold=';1' if bold else ''))


def download(url, directory, options=''):
    Popen('wget -q --show-progress {0} -N -P {1} {2}'.format(url, directory, options), shell=True).wait()
    return os.path.expanduser(directory + '/' + url.split("/")[-1])


def add_cmake_flag(flag):
    Popen('catkin config --profile native --remove-args {0};\
           catkin config --profile cross  --remove-args {0};\
           catkin config --profile native --append-args {0};\
           catkin config --profile cross  --append-args {0}'.format(flag), stdout=PIPE, shell=True).wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--catkin_ws',
                        default=os.getcwd(),
                        help='The workspace in which to enable the vendor library')
    parser.add_argument('-d', '--download',
                        action='store_true',
                        help='Only download & install libraries, do not enable them')
    parser.add_argument('-a', '--all',
                        action='store_true',
                        help='Install all available libraries')
    for library in libraries.keys():
        parser.add_argument('--{0}'.format(library),
                            action='store_true',
                            help='Install the {0} library'.format(library))

    args = parser.parse_args()

    # Check if there exists a valid catkin workspace
    p = Popen('cd {wd}; catkin locate'.format(wd=args.catkin_ws), shell=True, stdout=PIPE, stderr=PIPE)
    args.catkin_ws, _ = p.communicate()
    args.catkin_ws = args.catkin_ws.strip()
    if args.download:
        if not p.returncode == 0:
            log('\nDirectory {dir} is not a part of any catkin workspace!'.format(dir=args.catkin_ws), colors['red'])
            log('Please specify a valid workspace\n', colors['red'])
            exit(1)
        else:
            log('\nInstalling and enabling specified libraries in workspace {dir}'.format(dir=args.catkin_ws))
    else:
        log('\nDownloading specified libraries')

    # Iterate through every known library
    url_opener = urllib2.build_opener()
    url_opener.addheaders = [('User-Agent', 'Mozilla/5.0')]
    num_libs_success = 0
    num_libs_attempt = 0
    for library, json_url in libraries.items():
        if not args.all and not getattr(args, library):
            continue
        log('\nPreparing to install {0}...'.format(library), colors['magenta'], True)
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
                url = '/'.join([maven_url.rstrip('/'), dep['groupId'].replace('.', '/'),
                                dep['artifactId'], dep['version']])

                # Note: This is sloppy but at least it will cover the artifacts we need.
                # We will just allow wget to fail on nonexistant suffixes
                for suffix in ['headers', 'linuxathena', 'linuxathenastatic', 'linuxx86-64', 'llinuxx86-64static']:
                    name = '-'.join([dep['artifactId'], dep['version'], suffix])
                    download(url + '/' + name + '.zip', '~/frc2019/maven/' + dep['groupId'].replace('.', '/'))
            log('Finished downloading')

            # Add -D<library>=ON when compiling
            if not args.download:
                flag = '-D{0}'.format(library.upper())
                log('Enabling {0} in frc_control...'.format(flag))
                add_cmake_flag(flag + '=ON')
                log('Successfully enabled {0}'.format(flag))

            log('Finished installing {0} v{1}'.format(parsed['name'], parsed['version']), colors['green'])
            num_libs_success += 1
        except KeyboardInterrupt:
            log('Keyboard interrupt, aborting...', colors['yellow'])
            break
        except:
            log('Failed to install {0}!'.format(library), colors['red'])

    log('')
    if num_libs_attempt is 0:
        log('No libraries specified for install!\n', colors['red'], True)
        exit(1)
    elif num_libs_success != num_libs_attempt:
        log('{0}/{1} libraries installed sucessfully\n'.format(num_libs_success,
                                                               num_libs_attempt), colors['yellow'], True)
        exit(1)
    else:
        log('All specified libraries installed successfully\n', colors['green'], True)
        exit(0)
