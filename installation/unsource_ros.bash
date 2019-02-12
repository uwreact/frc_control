#!/bin/bash

if [ -z "$PS1" ] ; then
    echo -e "This script must be sourced. Use \"source unsource_ros.bash\" instead."
    exit
fi

cp ~/.bashrc ~/.bashrc_noros
sed -e '/source \/opt\/ros/ s/^#*/#/' -i ~/.bashrc_noros
exec $SHELL --rcfile ~/.bashrc_noros
