FROM ros:noetic

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    build-essential \
    wget \
    git \
    cmake\
    # gazebo11 \
    python3-catkin-tools \
    libnvidia-gl-515-server

# Create and setup user
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
WORKDIR /home/docker
RUN git clone https://github.com/PX4/PX4-Autopilot.git -b v1.13.1 --recursive && \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
ENV PX4_PATH=/home/docker/PX4-Autopilot
RUN echo hejooo
RUN rm -r $PX4_PATH/Tools/sitl_gazebo/models/iris
COPY models/iris $PX4_PATH/Tools/sitl_gazebo/models/iris
RUN sudo chmod 777 $PX4_PATH/Tools/sitl_gazebo/models/iris
RUN cd $PX4_PATH && DONT_RUN=1 make -j8 px4_sitl_default gazebo && cd ..

# Install ROS dependencies
RUN rosdep update && sudo apt-get update && \
    rosdep install --from-paths ${ROS_WORKSPACE} -r -y --ignore-src && \
    sudo apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-gazebo-ros \
    ros-noetic-cv-bridge

# Install GeographicLib datasets
RUN sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

SHELL ["/bin/bash", "-c"]

# Build
WORKDIR $ROS_WORKSPACE
RUN source /opt/ros/noetic/setup.bash && \
    catkin build

RUN echo "source /home/docker/ws/scripts/setup.sh" >> /home/docker/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]