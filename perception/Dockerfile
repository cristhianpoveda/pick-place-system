ARG BASE_IMAGE=ultralytics/ultralytics
ARG BASE_TAG=8.0.239
FROM ${BASE_IMAGE}:${BASE_TAG}

ENV DEBIAN_FRONTEND=noninteractive

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt update

COPY ./apt-requirements.txt /
RUN xargs apt-get install -y </apt-requirements.txt \
&& rm -rf /var/lib/apt/lists/*

ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
RUN echo PIP_INDEX_URL=${PIP_INDEX_URL}
COPY ./py-requirements.txt /
RUN python3 -m pip install  -r /py-requirements.txt

ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} cam
RUN adduser --gecos "CAM User" --disabled-password --uid ${UID} --gid ${GID} cam
RUN usermod -a -G dialout cam
ADD config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

ENV USER cam
USER cam

# create and build ros workspace
ENV HOME /home/${USER} 
RUN mkdir -p ${HOME}/ros_ws/src

WORKDIR ${HOME}/ros_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"
WORKDIR ${HOME}/ros_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make -DPYTHON_INCLUDE_DIR=/opt/conda/include/python3.10 -DPYTHON_EXECUTABLE=/opt/conda/bin/python3"

# set up environment
COPY config/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown cam /sbin/update_bashrc ; sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc
# Set entrypoint
COPY config/entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown cam /ros_entrypoint.sh ;

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/* 
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]