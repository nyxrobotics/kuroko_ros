#!/bin/bash

## Get script directory
SCRIPT_DIR=$(cd $(dirname $0); pwd)

## Enable catkin-tools
source `catkin locate --shell-verbs`

cd $SCRIPT_DIR
catkin source
cd ..

## Install dependency
contains() {
    [[ $1 =~ (^|[[:space:]])$2($|[[:space:]]) ]] && true || false
}

ignores="./robotis/robotis_framework/.robotis_framework.rosinstall, ./robotis/robotis_op3_tools/.robotis_op3_tools.rosinstall, ./robotis/robotis_op3/.robotis_op3.rosinstall"
ignore_files=${ignores//,/ }
files=`find . -type f -regextype posix-egrep -regex "\./.+\.rosinstall" | sort`
pre_n=0
n=`echo ${files} | wc -w`
while [ `comm -3 <(echo ${files}) <(echo ${ignore_files[@]} | sort) | wc -w` -ne 0 -a ${n} -ne ${pre_n} ]
do
    for f in ${files}
    do
        if `contains "${ignore_files}" ${f}` || [[ ${f} =~ ".github/" ]]; then
            echo ignore ${f}
        else
            vcs import --recursive --debug < ${f}
            ignore_files+=(${f})
        fi
    done
    files=`find . -type f -regextype posix-egrep -regex "\./.+\.rosinstall" | sort`
    pre_n=${n}
    n=`echo ${files} | wc -w`
done

rosdep update
rosdep install -r -y -i --from-paths .
catkin source
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
catkin build