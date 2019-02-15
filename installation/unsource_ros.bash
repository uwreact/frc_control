#!/bin/bash

echo "This script is currently broken!"
exit

if [ -z "$PS1" ] ; then
    echo -e "This script must be sourced. Use \"source unsource_ros.bash\" instead."
    exit
fi

cp ~/.bashrc ~/.bashrc_noros
sed -e '/source \/opt\/ros/ s/^#*/#/' -i ~/.bashrc_noros
exec -c $SHELL --rcfile ~/.bashrc_noros
