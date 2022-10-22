FROM ctumrs/mrs_uav_system:2022_w42

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    libnvidia-gl-515-server

# Setup user
RUN adduser --disabled-password --gecos '' docker && \
    # groupadd -r -g 110 render && \
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

SHELL ["/bin/bash", "-c"]

# Build
RUN source /opt/ros/noetic/setup.bash && \
    cd /opt/mrs/mrs_workspace && \
    sudo catkin build

WORKDIR $ROS_WORKSPACE

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/docker/.bashrc && \
    echo "source /home/docker/ws/devel/setup.bash" >> /home/docker/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> /home/docker/.bashrc && \
    echo "source /opt/mrs/mrs_workspace/devel/setup.bash" >> /home/docker/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]