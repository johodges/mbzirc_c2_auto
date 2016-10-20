# mbzirc_c2_auto

1. Install ubuntu 14.04 LTS with auto-install updates enables.
2. sudo apt-get update
3. sudo apt-get upgrade
4. Install ros indigo (instructions from: http://wiki.ros.org/indigo/Installation/Ubuntu)
    a. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    b. sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
    c. sudo apt-get update
    d. sudo apt-get install ros-indigo-desktop-full
    e. echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    f. source ~/.bashrc
    g. type "roscore" in terminal to confirm ros installation is working properly, ctl+c to kill it
5. Setup a catkin workspace (instructions from http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
    a. mkdir -p ~/catkin_ws/src
    b. cd ~/catkin_ws/src
    c. catkin_init_workspace
    d. cd ~/catkin_ws
    e. catkin_make
    f. source devel/setup.bash
    g. echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
6. Install ROS packages we use
    a. sudo apt-get install ros-indigo-husky-*
    b. sudo apt-get install ros-indigo-urg-*
    c. sudo apt-get install ros-indigo-teleop-*
    d. sudo apt-get install ros-indigo-rospy-*

7. Install our packages
    a. Copy mbzirc_c2 into ~/catkin_ws/src/
    b. cd ~/catkin_ws/src/mbzirc_c2/mbzirc_c2_auto/bin
    c. sudo chmod +x *.py
    d. sudo cp ~/catkin_ws/src/mbzirc_c2/mbzirc_c2_auto/sick_lms1xx.urdf.xacro /opt/ros/indigo/share/lms1xx/urdf
    e. cd ~/catkin_ws
    f. catkin_make
    g. source devel/setup.bash
    h. roslaunch mbzirc_ch2_auto ch2-sim.launch
