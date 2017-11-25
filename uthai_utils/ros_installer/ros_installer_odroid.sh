#!/bin/bash

name_catkinws=${name_catkinws:="catkin_ws"}
name_ros_distro=${name_ros_distro:="kinetic"}

version=`lsb_release -sc`

echo
echo
echo "====================================================================="
echo "============= Install Robot Operating System on Odroid =============="
echo "====================================================================="
echo "[ROS Kinetic ONLY supports Wily(Ubuntu 15.10) , Xenial(Ubuntu 16.04)]"
echo "Do you want to install ROS Kinetic (Y/n)"
read install
if [ "$install" == "N" ] || [ "$install" == "n" ]; then
    exit 0
fi

echo "[Checking the ubuntu version]"
echo "===> Ubuntu version is $version"
case $version in
    xenial)
        ;;
    willy)
        ;;
    *)
        echo "ERROR: ROS doesn't support Ubuntu $version "
        exit 0
        ;;
esac

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
    sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Set up your keys]"
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

echo "[Update & Upgrade the package]"
sudo apt -y update
sudo apt -y upgrade

echo "[Installing ROS]"
sudo apt-get -y install ros-$name_ros_distro-desktop


echo "[rosdep init and python-rosinstall]"
if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo sh -c "rosdep init"
fi

rosdep update
. /opt/ros/$name_ros_distro/setup.sh

echo "[Setting the ROS environment]"
echo "source /opt/ros/$name_ros_distro/setup.bash" >> ~/.bashrc
echo "source ~/$name_catkinws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
source ~/.bashrc

echo "[Install catkin tool and rosinstall]"
sudo apt -y install python-catkin-tools python-rosinstall

echo "[Make system using CMake]"
sudo apt -y install build-essential

echo "[Install common dependencies]"
# System tools
sudo apt -y install vim

# Formatting utilities
sudo apt -y install clang-format-4.0 python-flake8

echo "[Set up catkin workspace]"
if [ ! -e ~/$name_catkinws/src/CMakeLists.txt ]
then
	echo "Generating catkin workspace at ~/$name_catkinws"
	mkdir -p ~/$name_catkinws/src
    cd ~/$name_catkinws/src
    catkin_init_workspace
    cd ~/$name_catkinws
    catkin_make
else
    echo "Using existing catkin workspace at ~/$name_catkinws"
fi

echo
echo "========================================================="
echo "============= ROS Installation Complete!!! =============="
echo "========================================================="
echo

exec bash
exit 0