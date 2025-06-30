#!/bin/bash

# copied from https://github.com/ros-planning/moveit_ci/blob/master/check_clang_format.sh
# This directory can have its own .clang-format config file but if not, MoveIt's will be provided
if [ ! -f .clang-format ]; then
    wget "https://raw.githubusercontent.com/ros-planning/moveit/melodic-devel/.clang-format"
fi

# Run clang-format
echo "Running clang-format check"
find . -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format-3.9 -i -style=file

echo "Showing changes in code style:"
git --no-pager diff

# Make sure no changes have occured in repo
if ! git diff-index --quiet HEAD --; then
    # changes
    echo "clang-format test failed: changes required to comply to formatting rules. See diff above."
    exit 1 # error
fi

echo "Passed clang-format test"
