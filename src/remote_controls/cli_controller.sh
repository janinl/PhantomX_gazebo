#!/bin/bash

speed=0
maxSpeed=5
rosTopicSpeed=/webbie1/cmd_abs_speed
rosTopicTurn=/webbie1/cmd_add_rel_turn

while [ 1 ] ; do

 turn=
 read -N 1 -p "Ready for keypress..." key
 case $key in
  w) speed=$(( speed+1 ));;
  x) speed=$(( speed-1 ));;
  a) turn=-22.5;;
  d) turn=22.5;;
 esac

 speed=$(( speed > maxSpeed ? maxSpeed:speed ))
 speed=$(( speed < -maxSpeed ? -maxSpeed:speed ))
 echo
 date
 echo speed=$speed  \t turn=$turn

 if [[ -z "$turn" ]] ; then
  timeout 1s rostopic pub $rosTopicSpeed std_msgs/Float64 -- $speed
 else
  timeout 1s rostopic pub $rosTopicTurn std_msgs/Float64 -- $turn
 fi

done
