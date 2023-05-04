FROM ros:noetic

# Download and install dependencies for OPPT
RUN apt-get update && \
    apt-get install -y git && \
    cd ~ && \
    git clone https://github.com/RDLLab/oppt && \
    cd oppt && \
    chmod +x install_dependencies.sh && ./install_dependencies.sh


# Extra dependencies that are needed.
RUN apt install -y ros-noetic-nlopt libnlopt-cxx-dev wget ros-noetic-rviz ros-noetic-kdl-parser ros-noetic-trac-ik ros-noetic-trac-ik-lib && \
    rm -rf /var/lib/apt/lists/*

# We need a specific version of FCL, so we will build from scratch
RUN apt remove -y ros-noetic-fcl libfcl-dev && \
    cd ~ && \
    git clone https://github.com/flexible-collision-library/fcl && \
    cd fcl && git checkout fcl-0.5 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    sudo make install

# Install spatialindex
RUN wget http://download.osgeo.org/libspatialindex/spatialindex-src-1.8.5.tar.gz && \
    tar zxfv spatialindex-src-1.8.5.tar.gz && \ 
    cd spatialindex-src-1.8.5 && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j2 && sudo make install

# Finally, make and install oppt
RUN . /opt/ros/noetic/setup.sh && \
    cd ~/oppt/src/ && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=. .. && \
    make && make install && \
    echo 'source /root/oppt/src/build/share/oppt/setup.sh' >> ~/.bashrc 