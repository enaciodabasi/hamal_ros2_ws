#!/bin/bash

user=@USER@
ros_domain_id=@DOMAIN_ID@
workspace_path=@WORKSPACE_PATH@
package_name=@PACKAGE_NAME@
launch_file_name=@LAUNCH_FILE_NAME@

which setpriv > /dev/null
if [ "$?" != "0" ]; then
  echo "Could not find setpriv package"
  exit 1
fi

## Source setup file
if [ -e ${workspace_path}/install/setup.bash ]; then
  . ${workspace_path}/install/setup.bash 
else
  echo "Could not source ROS 2 files."
  exit 1;
fi

## Check if realtime priority is equal to 99
## if not try to set it 
if [ ! $(ulimit -r) -eq 99 ]; then
  ulimit -r 99
  if [ "$?" != "0" ]; then
    echo "Could not set realtime priority."
    exit 1
  fi
  echo "Set realtime priority to 99."
fi

export ROS_DOMAIN_ID=$ros_domain_id

setpriv --reuid @(user) ros2 launch $package_name $launch_file_name
ROS_LAUNCH_PID=$!

wait $ros_launch_pid