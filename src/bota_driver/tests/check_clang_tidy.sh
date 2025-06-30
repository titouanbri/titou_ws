#!/bin/bash

# This directory can have its own .clang-tidy config file but if not, MoveIt's will be provided
if [ ! -f .clang-tidy ]; then
    wget "https://raw.githubusercontent.com/ros-planning/moveit/melodic-devel/.clang-tidy"
fi

export CATKIN_WS=/root/catkin_ws
# Run clang-tidy
echo "Running clang-tidy check"
i=0
for file in $(find $CATKIN_WS/build -name compile_commands.json); do
    fixes_file="fixes-$i.yaml"
    run-clang-tidy -header-filter="$CATKIN_WS/src/bota_driver/.*" -export-fixes $fixes_file -p $(dirname $file)
    i=$((i + 1))
done

linting_passed=true
for file in $(ls $CATKIN_WS/src/bota_driver/fixes-*.yaml); do
    if [ -s $file ]; then
        if (cat $file | grep "bota_driver"); then

            echo "The fixes file $file has some linting issues:"
            cat $file
            linting_passed=false
        else
            echo "The fixes file $file contains linting issues from other packages"
        fi
    else
        echo "The fixes file $file is empty."
    fi
done
if [ "$linting_passed" = true ]; then
    echo "Passed clang-tidy test"
else
    echo "clang-tidy test failed: changes required to comply linting rules. See issues above."
    exit 1
fi
