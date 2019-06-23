#!/bin/bash

speed=0
maxSpeed=10
rosTopicSpeed=/webbie1/cmd_abs_speed
rosTopicTurn=/webbie1/cmd_add_relative_rotation

while [ 1 ] ; do

 oldSpeed=$speed
 turn=
 while read -t 0.1 -N 1 -p "." key ; do
  case $key in
   w) speed=$(( speed+1 ));;
   x) speed=$(( speed-1 ));;
   s) speed=0;;
   a) turn=$(( turn-22 ));;
   d) turn=$(( turn+22 ));;
   *) exit
  esac
 done

 if [[ -z "$turn" ]] && [[ "$speed" == "$oldSpeed" ]] ; then
  sleep 1s
  continue
 fi

 speed=$(( speed > maxSpeed ? maxSpeed:speed ))
 speed=$(( speed < -maxSpeed ? -maxSpeed:speed ))
 echo
 date
 echo speed=$speed  \t turn=$turn

 if [[ "$speed" != "$oldSpeed" ]] ; then
  timeout 2s rostopic pub $rosTopicSpeed std_msgs/Float64 -- $speed
 fi
 if [[ ! -z "$turn" ]] ; then
  timeout 2s rostopic pub $rosTopicTurn std_msgs/Float64 -- $turn
 fi

done
