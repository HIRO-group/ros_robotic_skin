FROM osrf/ros:melodic-desktop-full-bionic
# FROM ros:melodic-ros-base
# install the necessary packages
RUN apt-get update
RUN apt-get install wget
RUN sudo apt install -y python3-pip
RUN mkdir -p /root/catkin_ws/src/ros_robotic_skin
COPY . /root/catkin_ws/src/ros_robotic_skin
RUN cd /root/catkin_ws/src && git clone https://github.com/erdalpekel/panda_simulation.git \
    && git clone https://github.com/erdalpekel/panda_moveit_config.git \
    && git clone --branch simulation https://github.com/erdalpekel/franka_ros.git \
    && cd .. \
    && sudo apt-get install libboost-filesystem-dev \
    && rosdep install --from-paths src --ignore-src -y -r --skip-keys libfranka \
    && cd src \
    && wstool init \
    && wstool merge https://gist.githubusercontent.com/jarvisschultz/f65d36e3f99d94a6c3d9900fa01ee72e/raw/sawyer_packages.rosinstall \
    && wstool update \
    && sudo apt-get install -y ros-melodic-libfranka \
    && cd .. \
    && rosdep install --from-paths src --ignore-src -y -r --skip-keys libgazebo7-dev \
    && cd src/sawyer_simulator/sawyer_sim_controllers \
    && rm CMakeLists.txt \
    && wget https://gist.githubusercontent.com/peasant98/5d1f1e6ee23d909f406995846dfffb50/raw/23a9a9ae16b42093f7449a9670ba41fda9809c32/CMakeLists.txt

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /root/catkin_ws; catkin_make'
RUN /bin/bash -c 'source /root/catkin_ws/devel/setup.bash'
RUN echo 'source /root/catkin_ws/devel/setup.bash' >> ~/.bashrc
RUN sed -i "\$i source /root/catkin_ws/devel/setup.bash" ros_entrypoint.sh
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


