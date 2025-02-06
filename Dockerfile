# Используем образ ROS Noetic как базовый
FROM osrf/ros:noetic-desktop-full
LABEL authors="RegisLab"

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && apt-get install -y \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libgl1-mesa-dev


# Install git
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y git

# Clone and build PX4
RUN git clone https://github.com/PX4/PX4-Autopilot.git
WORKDIR PX4-Autopilot
RUN git checkout v1.13.3
RUN apt-get install -y wget
RUN /bin/bash Tools/setup/ubuntu.sh
RUN DONT_RUN=1 make px4_sitl_default gazebo -j$(nproc)

# install mavros
RUN apt-get install ros-noetic-mavros -y
RUN apt-get install ros-noetic-mavros-extras -y
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN bash ./install_geographiclib_datasets.sh -y
RUN rm install_geographiclib_datasets.sh


# add user
RUN useradd -ms /bin/bash docker && echo "docker:docker" | chpasswd && adduser docker sudo
RUN usermod -aG sudo docker
RUN echo 'docker ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers


USER docker
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/docker/.bashrc
WORKDIR /home/docker

#install catkin
RUN mkdir -p /home/docker/catkin_ws/src
WORKDIR /home/docker/catkin_ws/src
RUN sudo apt-get install python3-catkin-tools -y
RUN sudo apt-get install nano -y
RUN git clone https://github.com/ros/catkin.git


ENTRYPOINT ["top", "-b"]
