#!/bin/sh

# TODO(matthew-reynolds): Replace with a python script that gives better output.
# clang-format's output is particularily bad - We simply output the number of
# changes required, rather than actually showing what/where the changes are
# required. Additionally, we should consider running through all the checks
# before erroring out so that all required changes can be addressed at once.

echo "Checking C++ code with clang-tidy"
command -v clang-tidy-7 || apt install clang-tidy-7 -y -qq --no-install-recommends
workspace=$(catkin locate 2> /dev/null)
if [ $? -eq 0 ]; then
    ./run_clang_tidy.py frc_control --verbose
else
    ./run_clang_tidy.py -w /root/catkin_ws frc_control --verbose
fi

if [ $? -ne 0 ]; then
    echo >&2 "Code does not meet style requirements!"
    exit 1
fi
echo "clang-tidy passed successfully\n"


echo "Checking C++ code formatting with clang-format"
command -v clang-format-7 || apt install clang-format-7 -y -qq --no-install-recommends
changes_required=$(find . -name "*.h" -o -name "*.cpp" |\
                   xargs clang-format-7 -style=file -output-replacements-xml |\
                   grep -c "<replacement ")
if [ $changes_required -ne 0 ]; then
    echo >&2 "Code does not meet formatting requirements, $changes_required change(s) required!"
    echo >&2 "Please run clang-format!"
    exit 1
fi
echo "clang-format passed successfully\n"


echo "Checking python code style with pylint..."
command -v pylint || pip install pylint
find . -iname "*.py" -o -iregex ".*/scripts/.*" | xargs pylint
if [ $? -ne 0 ]; then
    echo >&2 "Code does not meet style requirements!"
    exit 1
fi
echo "Pylint passed successfully!\n"


echo "Checking python code style with yapf..."
command -v yapf || pip install yapf
yapf --diff --recursive .
if [ $? -ne 0 ]; then
    echo >&2 "Code does not meet style requirements! Please run yapf to format the code."
    exit 1
fi
echo "Yapf passed successfully!\n"


echo "All checks passed successfully, code meets style requirements!"
exit 0
