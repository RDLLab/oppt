#!/bin/bash

res="succ"
PKGS=""
BASE_PKG=""

command_exists() {
    type "$1" &> /dev/null ;
}

# Check if command in $1 exists.
# If not, add $2 to the list of installed libraries
checkCMDAndAppend() {
    if ! command_exists $1
    then
       echo $1
       BASE_PKG="$BASE_PKG $2"
    else 
       echo "$2 is already installed. Skipping..."
    fi
}

# Check if package in $1 exists and can be found by pkg-config. 
# If not, add $2 to the list of installed libraries
checkPKGAndAppend() {
    $(pkg-config --exists $1) &> /dev/null    
    if [ $? -eq 0 ]; then
        echo "$1 found by pkg-config. Skipping..."
    else        
        PKGS="$PKGS $2"
    fi 
}

# Check if package $1 can be found by dpkg. 
# If not, add it to the list of installed packages
checkAndAppend() {
    dpkg -s $1 &> /dev/null
    if [ $? -eq 0 ]; then
        echo "$1 already installed. Skipping..."
    else
        PKGS="$PKGS $1"
    fi  
}

# Check the current status of the script
checkStatus() {
    if [ $res != "succ" ]; then
        echo "Installation failed"
        exit
    fi
}

# Detect Ubuntu version
detect_ubuntu_version() {    
    ARCH=$(uname -m)  
    if [ $ARCH != "x86_64" ]
    then
       echo "Incompatible architecture $ARCH"
       res="fail"
       return
    fi
        
    UBUNTU_VERSION=$(lsb_release -rs)    
    UBUNTU_MAJOR_VERSION=$(echo $UBUNTU_VERSION | grep -oP '.*?(?=\.)')
    if [ $UBUNTU_MAJOR_VERSION -lt 15 ]
    then
        echo "Your Ubuntu version is too old. OPPT requires Ubuntu 15.10 (Wily) or higher"
        res="fail"
        return
    fi

    GAZEBO_SOURCES="http://packages.osrfoundation.org/gazebo/ubuntu-stable"
    GAZEBO_PKG="gazebo-stable.list"
    GZ_VERSION=7
 
    if [ $UBUNTU_MAJOR_VERSION -eq 15 ]
    then       
       ROS_DISTR="jade"       
       return
    fi
        
    if [ $UBUNTU_MAJOR_VERSION -eq 16 ]
    then
       ROS_DISTR="kinetic"       
       return
    fi
    
    if [ $UBUNTU_MAJOR_VERSION -eq 17 ]
    then
       ROS_DISTR="melodic"       
       GZ_VERSION=9
       return
    fi
    
    if [ $UBUNTU_MAJOR_VERSION -eq 18 ]
    then
       ROS_DISTR="melodic"       
       GZ_VERSION=9
       return
    fi
    
    if [ $UBUNTU_MAJOR_VERSION -eq 19 ]
    then
       ROS_DISTR="melodic"
       GZ_VERSION=9
       return
    fi
    
    if [ $UBUNTU_MAJOR_VERSION -ge 20 ]
    then
       ROS_DISTR="noetic"
       GZ_VERSION=9
       return
    fi     
}

install_common_dependencies() {
    checkAndAppend build-essential
    checkCMDAndAppend git git
    checkCMDAndAppend cmake cmake 
    checkCMDAndAppend hg mercurial
    checkCMDAndAppend pkg-config pkg-config
    echo "Installing base packages"    
    if [ -n "$BASE_PKG" ]; then
        sudo apt-get -y install $BASE_PKG
        if [[ $? > 0 ]]; then
           res="fail"
           return
        fi
        sudo ldconfig
    else
        echo "Required base packages already installed"        
    fi   

    echo "Installing remaining packages"    
    checkAndAppend libboost-all-dev
    checkAndAppend libtinyxml-dev
    checkPKGAndAppend eigen3 libeigen3-dev
    checkPKGAndAppend assimp libassimp-dev
    checkPKGAndAppend fcl libfcl-dev
    checkAndAppend gazebo${GZ_VERSION}
    checkAndAppend libgazebo${GZ_VERSION}-dev   
    
    if [ -n "$PKGS" ]; then
        #echo "Packages being installed: $PKGS"
        sudo apt-get -y install $PKGS
        if [[ $? > 0 ]]; then
           res="fail"
           return
        fi
    else
        echo "Required base packages already installed"
    fi    
}

install_ros() {
    # Check for components of ROS
    ROS_PACKAGES=""
    ROS_IK_PACKAGES=""
    ADD_KEY=true    
    if command_exists rospack
    then
       ADD_KEY=false
       rv=$(rospack find rviz)
       if [[ $rv != *"rviz"* ]]; then
          echo "Install ros-$ROS_DISTR-rviz"
          ROS_PACKAGES="$ROS_PACKAGES ros-$ROS_DISTR-rviz"
       fi 
       rv=$(rospack find kdl-parser) 
       if [[ $rv != *"kdl-parser"* ]]; then 
          echo "Install ros-$ROS_DISTR-kdl-parser"          
          ROS_IK_PACKAGES="$ROS_IK_PACKAGES ros-$ROS_DISTR-kdl-parser"
       fi
       rv=$(rospack find trac-ik) 
       if [[ $rv != *"kdl-parser"* ]]; then 
          echo "Install ros-$ROS_DISTR-trac-ik"          
          ROS_IK_PACKAGES="$ROS_IK_PACKAGES ros-$ROS_DISTR-trac-ik ros-$ROS_DISTR-trac-ik-lib"
       fi   
    else
       ROS_PACKAGES="ros-$ROS_DISTR-ros-base ros-$ROS_DISTR-rviz"
       ROS_IK_PACKAGES="ros-$ROS_DISTR-kdl-parser ros-$ROS_DISTR-trac-ik ros-$ROS_DISTR-trac-ik-lib"
    fi

    # Installs the ros-base variant of ROS plus the visualization stack, kdl-parser and trac-ik
    if [ "$ADD_KEY" = true ]; then
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'    
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    fi
       
    sudo apt-get update    
    sudo apt-get -y install $ROS_PACKAGES
    sudo apt-get -y install $ROS_IK_PACKAGES
    source /opt/ros/$ROS_DISTR/setup.bash    
}

install_libspatialindex() {
    # Check if libspatialindex is already installed
    sudo ldconfig
    siOut=$(ldconfig -p | grep libspatialindex)
    if [ `echo $siOut | grep -c "libspatialindex" ` -gt 0 ]
	then
	    echo "libspatialindex is already installed. Skipping..."
  		return
	fi
    # Downloads, compiles and installs libspatialindex-1.8.5
    wget http://download.osgeo.org/libspatialindex/spatialindex-src-1.8.5.tar.gz
    tar zxfv spatialindex-src-1.8.5.tar.gz
    cd spatialindex-src-1.8.5
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
    make -j2 && sudo make install
    cd ../../
    rm -rf spatialindex-src-1.8.5
    rm spatialindex-src-1.8.5.tar.gz
}

USE_ROS=false
if [ -z $1 ]
then
  echo "Installing without ROS"
else  
  if [ $1 = "--use-ros" ]
  then
      echo "Installing with ROS"
      USE_ROS=true
  else
      echo "Command line option '$1' not recognized. Available option: '--use-ros'"
      exit
  fi
fi

detect_ubuntu_version
checkStatus
install_common_dependencies
checkStatus
if [ "$USE_ROS" = true ]
then   
   install_ros
fi
checkStatus
install_libspatialindex
checkStatus
echo "Done."
