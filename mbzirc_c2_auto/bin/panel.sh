#!/bin/bash   
#
x=50.333
y=-20.65
Y=0

z=0.05
R=0
P=0
z1=$(bc <<<"$z+0.6225")
c1=$(echo "c($Y)" | bc -l)
s1=$(echo "s($Y)" | bc -l)

rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/mbzirc_c2_auto/mbzirc_c2_auto/gazebo_models/models/panel/model.sdf -sdf -model panel -x $x -y $y -z $z -R $R -P $P -Y $Y

l=0.168
w=0.2

x1=$(bc <<<"$x-($s1)*$w+($c1)*$l")
y1=$(bc <<<"$y+($s1)*$l+($c1)*$w")

rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/mbzirc_c2_auto/mbzirc_c2_auto/gazebo_models/models/wrench12/model.sdf -sdf -model wrench12 -x $x1 -y $y1 -z $z1 -R $R -P $P -Y $Y

w=0.25

x2=$(bc <<<"$x-($s1)*$w+($c1)*$l")
y2=$(bc <<<"$y+($s1)*$l+($c1)*$w")

rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/mbzirc_c2_auto/mbzirc_c2_auto/gazebo_models/models/wrench15/model.sdf -sdf -model wrench15 -x $x2 -y $y2 -z $z1 -R $R -P $P -Y $Y

w=0.3

x3=$(bc <<<"$x-($s1)*$w+($c1)*$l")
y3=$(bc <<<"$y+($s1)*$l+($c1)*$w")

rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/mbzirc_c2_auto/mbzirc_c2_auto/gazebo_models/models/wrench13/model.sdf -sdf -model wrench13 -x $x3 -y $y3 -z $z1 -R $R -P $P -Y $Y

w=0.35

x4=$(bc <<<"$x-($s1)*$w+($c1)*$l")
y4=$(bc <<<"$y+($s1)*$l+($c1)*$w")

rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/mbzirc_c2_auto/mbzirc_c2_auto/gazebo_models/models/wrench18/model.sdf -sdf -model wrench18 -x $x4 -y $y4 -z $z1 -R $R -P $P -Y $Y

w=0.4

x5=$(bc <<<"$x-($s1)*$w+($c1)*$l")
y5=$(bc <<<"$y+($s1)*$l+($c1)*$w")

rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/mbzirc_c2_auto/mbzirc_c2_auto/gazebo_models/models/wrench19/model.sdf -sdf -model wrench19 -x $x5 -y $y5 -z $z1 -R $R -P $P -Y $Y

w=0.45

x6=$(bc <<<"$x-($s1)*$w+($c1)*$l")
y6=$(bc <<<"$y+($s1)*$l+($c1)*$w")

rosrun gazebo_ros spawn_model -file ~/catkin_ws/src/mbzirc_c2_auto/mbzirc_c2_auto/gazebo_models/models/wrench14/model.sdf -sdf -model wrench14 -x $x6 -y $y6 -z $z1 -R $R -P $P -Y $Y

