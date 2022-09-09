FROM ros:noetic

RUN apt-get update && apt-get -y install \
    build-essential \
    wget \
    gazebo11

# setup user
RUN groupadd -r -g 110 render \
    && adduser --disabled-password --gecos '' docker \
    && adduser docker sudo \
    && adduser docker render \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER docker
WORKDIR /home/docker