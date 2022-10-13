FROM ros:noetic

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    build-essential \
    wget \
    git \
    cmake\
    gazebo11 \
    python3-catkin-tools \
    libnvidia-gl-515-server

# Setup user
RUN adduser --disabled-password --gecos '' docker && \
    groupadd -r -g 110 render && \
    adduser docker sudo && \
    adduser docker render && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER docker

# Create ROS workspace
WORKDIR /home/docker
RUN mkdir -p ws/src
WORKDIR ws/src
ENV ROS_WORKSPACE=/home/docker/ws
RUN catkin init

# Add PX4
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive && \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

# Install ROS dependencies
RUN rosdep update && sudo apt-get update && \
    rosdep install --from-paths ${ROS_WORKSPACE} -r -y --ignore-src && \
    sudo apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-gazebo-ros \
    ros-noetic-cv-bridge

# Build and Install mavlink_sitl_gazebo
# RUN git clone https://github.com/PX4/PX4-SITL_gazebo.git --recursive && \
#     cd PX4-SITL_gazebo && mkdir build && cd build && \
#     cmake .. && make -j4 && \
#     sudo make install && \
#     cd .. && rm -rf PX4-SITL_gazebo

# Install GeographicLib datasets
RUN sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/docker/.bashrc && \
    echo "source /home/docker/ws/devel/setup.bash" >> /home/docker/.bashrc
    # echo "export ROS_PACKAGE_PATH=\"${ROS_PACKAGE_PATH}:/home/docker/ws/src/PX4-Autopilot\"" >> /home/docker/.bashrc && \
    # echo "export ROS_PACKAGE_PATH=\"${ROS_PACKAGE_PATH}:/home/docker/ws/src/PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo\"" >> /home/docker/.bashrc

# Build
WORKDIR $ROS_WORKSPACE
RUN source /opt/ros/noetic/setup.bash && \
    catkin build

RUN sudo apt-get -y --quiet --no-install-recommends install libnvidia-gl-515-server 

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]