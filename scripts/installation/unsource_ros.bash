#!/bin/bash

echo "This script is currently broken!"
echo "Please manually unsource ROS before cross-compiling."
exit

if [ -z "$PS1" ] ; then
    echo "This script must be sourced rather than executed."
    echo "Use \"source unsource_ros.bash\" instead."
    exit
fi

VARS=$(env | cut -d= -f1)
for VAR in $VARS; do
    if [[ $VAR == "ROS"* ]]; then
        unset $VAR
    fi
done
