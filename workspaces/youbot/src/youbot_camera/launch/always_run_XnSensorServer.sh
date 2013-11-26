#!/bin/bash

if [[ $XN_SERVER_LOCATION = *XnSensorServer ]]
then

  #runs XnSensorServer while roslaunch is running
  while pidof -sx roslaunch > /dev/null
  do
    ${XN_SERVER_LOCATION:-~/xtion/Sensor-Bin-Linux32-v5.0.2.3/Bin/XnSensorServer}
  done

else
  echo 'XN_SERVER_LOCATION does not seem to point to XnSensorServer'
  echo 'do:  export XN_SERVER_LOCATION=locationofXnSensorServer'
  echo 'and make sure XnSensorServer has execute permissions'
  exit 0
fi
