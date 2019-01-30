#!/bin/bash
# Extract maven archives

function extract {
  path=${1}
  pkg=${2}

  # Get the latest version
  ar=(${path}/${pkg}/*)
  IFS=$'\n'
  version=$(basename `echo "${ar[*]}" | sort -nr | head -n1`)

  unzip -oq ${path}/${pkg}/${version}/${pkg}-${version}-headers.zip -d ${extracted}/include/${pkg}
  if [ -f ${path}/${pkg}/${version}/${pkg}-${version}-sources.zip ]; then
    unzip -oq ${path}/${pkg}/${version}/${pkg}-${version}-sources.zip -d ${extracted}/src/${pkg}
  fi
  if [ -f ${path}/${pkg}/${version}/${pkg}-${version}-linuxx86-64\*.zip ]; then
    unzip -oj ${path}/${pkg}/${version}/${pkg}-${version}-linuxx86-64\*.zip linux/x86-64/* -d ${extracted}/lib_x86/${pkg} 2> /dev/null
  fi
  unzip -oj ${path}/${pkg}/${version}/${pkg}-${version}-linuxathena\*.zip linux/athena/* -d ${extracted}/lib_athena/${pkg} 2> /dev/null
}

function extract-cpp {
  pkg=${1}

  # Get the latest version
  ar=(${pkg}/${pkg}-cpp/*)
  IFS=$'\n'
  version=$(basename `echo "${ar[*]}" | sort -nr | head -n1`)

  unzip -oq ${pkg}/${pkg}-cpp/${version}/${pkg}-cpp-${version}-headers.zip -d ${extracted}/include/${pkg}
  unzip -oq ${pkg}/${pkg}-cpp/${version}/${pkg}-cpp-${version}-sources.zip -d ${extracted}/src/${pkg}
  unzip -oj ${pkg}/${pkg}-cpp/${version}/${pkg}-cpp-${version}-linuxx86-64\*.zip linux/x86-64/* -d ${extracted}/lib_x86/${pkg} 2> /dev/null
  unzip -oj ${pkg}/${pkg}-cpp/${version}/${pkg}-cpp-${version}-linuxathena\*.zip linux/athena/* -d ${extracted}/lib_athena/${pkg} 2> /dev/null
}

extracted=~/frc2019/extracted
rm -r $extracted
for dir in $extracted  $extracted/include  $extracted/src  $extracted/lib_x86  $extracted/lib_athena; do
  mkdir $dir
done

cd ~/frc2019/maven/edu/wpi/first

extract-cpp cameraserver
extract-cpp cscore
extract-cpp hal
extract-cpp ntcore
extract-cpp wpilibc
extract-cpp wpiutil

extract ni-libraries chipobject
extract ni-libraries netcomm
extract thirdparty/frc2019 googletest

cd thirdparty/frc2019; extract-cpp opencv
