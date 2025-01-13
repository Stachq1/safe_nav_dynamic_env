FROM ros:humble

# Set the working directory
WORKDIR /root

# Copy the repository into the container
COPY . /root/safe_nav_dynamic_env

# Update the package list and install dependencies
RUN apt-get update && apt-get install -y \
    ca-certificates wget \
    libgoogle-glog-dev \
    libogre-1.9-dev \
    libgflags-dev \
    libyaml-cpp-dev \
    libboost-system-dev \
    libboost-thread-dev \
    libxmlrpcpp-dev \
    protobuf-compiler \
    libpcl-dev \
    vim \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-rviz2 \
    ros-humble-tf-transformations \
    python3-pip

# Install the pip packages related to SPOT
RUN python3 -m pip install --upgrade bosdyn-client bosdyn-mission bosdyn-choreography-client bosdyn-orbit

# Allow rviz to find the Ogre libraries
RUN export LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:$LD_LIBRARY_PATH && \
    export LD_LIBRARY_PATH=/opt/drake/lib:$LD_LIBRARY_PATH

# Remove apt installed Eigen 3.4.0 and install 3.3.7 instead
RUN rm -rf /usr/include/eigen3 && mv /root/safe_nav_dynamic_env/eigen3 /usr/include/

# Source the ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]
