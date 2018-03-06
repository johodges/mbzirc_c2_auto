# mbzirc_c2_auto

1. Install ubuntu 14.04 LTS with auto-install updates enabled.
2. sudo apt-get update
3. sudo apt-get upgrade
4. Install ros indigo (instructions from: http://wiki.ros.org/indigo/Installation/Ubuntu)
    * sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    * sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
    * sudo apt-get update
    * sudo apt-get install ros-indigo-desktop-full
    * echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    * source ~/.bashrc
    * type "roscore" in terminal to confirm ros installation is working properly, ctl+c to kill it
5. Setup a catkin workspace (instructions from http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
    * mkdir -p ~/catkin_ws/src
    * cd ~/catkin_ws/src
    * catkin_init_workspace
    * cd ~/catkin_ws
    * catkin_make
    * source devel/setup.bash
    * echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
6. Install ROS packages we use
    * sudo apt-get install ros-indigo-husky-*
    * sudo apt-get install ros-indigo-urg-*
    * sudo apt-get install ros-indigo-teleop-*
    * sudo apt-get install ros-indigo-rospy-*
    * sudo apt-get install git
    * sudo apt-get install ros-indigo-trac-ik*
    * sudo apt-get install ros-indigo-smach*
    * sudo dpkg -i libassimp3_3.0~dfsg-2_amd64.deb
    * sudo apt-get install ros-indigo-moveit-python ros-indigo-moveit-commander 
    * sudo apt-get install xdot
    * sudo apt-get install ros-indigo-moveit*
    * sudo apt-get install ros-indigo-velodyn*
    * sudo apt-get install ros-indigo-usb-cam

7. Install our packages
    * cd ~/catkin_ws/src
    * git clone https://github.com/johodges/mbzirc_c2_auto
    * cd ~/catkin_ws/src/mbzirc_c2_auto/mbzirc_c2_auto/bin
    * sudo chmod +x *.py
    * sudo cp ~/catkin_ws/src/mbzirc_c2_auto/mbzirc_c2_auto/sick_lms1xx.urdf.xacro /opt/ros/indigo/share/lms1xx/urdf
    * sudo cp ~/catkin_ws/src/mbzirc_c2_auto/mbzirc_c2_auto/xdot.py /opt/ros/indigo/lib/python2.7/dist-packages/xdot/xdot.py
    * cd ~/catkin_ws
    * catkin_make
    * source devel/setup.bash

8. Modifying the panel
    * Changing color: modify lines 29-41 in: mbzirc_c2_auto/mbzirc_c2_auto/gazebo_models/models/panel/model.sdf
    * Changing position/orientation: modify x, y, Y in: mbzirc_c2_auto/mbzirc_c2_auto/bin/panel.sh

9. Running with SMACH:
    * roslaunch mbzirc_c2_auto ch2-smach.launch

10. ROS-Kinetic (alpha instructions)
    * sudo apt-get update
    * sudo apt-get upgrade
    * sudo apt-get install ros-kinetic-moveit* ros-kinetic-manipulation-msgs ros-kinetic-joint-* ros-kinetic-trac-ik* ros-kinetic-urg-* ros-kinetic-teleop-t* ros-kinetic-rospy* ros-kinetic-smach* ros-kinetic-velodyne* ros-kinetic-usb-cam ros-kinetic-ur* ros-kinetic-lms1xx ros-kinetic-multimaster-*  ros-kinetic-interactive-marker-twist-server ros-kinetic-gazebo-* ros-kinetic-twist-mux ros-kinetic-pointcloud-to* xdot

    * mkdir ~/catkin_ws_mbzirc2017 && cd ~/catkin_ws_mbzirc2017
    * mkdir src && cd src
    * git clone https://github.com/johodges/mbzirc_c2_auto -b kinetic-devel
    * git clone https://github.com/husky/husky -b kinetic-devel
    * git clone https://github.com/johodges/mbzirc_c2_simulation -b kinetic-devel
    * cd .. && catkin_make

11. Open new terminal and test husky install
    * export HUSKY_URDF_EXTRAS=$(rospack find husky_description)/urdf/empty.urdf
    * roslaunch husky_gazebo husky_empty_world.launch

12. Open new terminal and launch mbzirc 2017 simulation
    * roslaunch mbzirc_c2_auto ch2-smach.launch
    * press spacebar in the terminal once everything has loaded to start. Note, the husky cannot currently turn with the ur5 mounted in kinetic.


