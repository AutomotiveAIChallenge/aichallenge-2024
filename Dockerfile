FROM osrf/ros:humble-desktop AS common

RUN apt-get update
RUN apt-get -y install libgl1-mesa-glx libgl1-mesa-dri
RUN apt-get -y install iproute2
RUN apt-get -y install ros-humble-rmw-cyclonedds-cpp

FROM common AS dev

ENV ROS_LOCALHOST_ONLY 1
ENV RCUTILS_COLORIZED_OUTPUT 1

FROM common AS eval

ENV RCUTILS_COLORIZED_OUTPUT 0

#RUN mkdir /ws
#RUN git clone --depth 1 https://github.com/AutomotiveAIChallenge/aichallenge-source-materials /ws/aichallenge2023-racing

# Copy into Container
#COPY AWSIM /ws/AWSIM
#COPY aichallenge_submit.tar.gz /ws
#COPY main.bash /ws

# Organize Files for Execution (Copy user files and map data to the executable folder)
#RUN cp -r /ws/aichallenge2023-racing/aichallenge /aichallenge
#RUN cp -r /ws/AWSIM /aichallenge
#RUN chmod 757 /aichallenge
#RUN rm -rf /aichallenge/autoware/src/aichallenge_submit
#RUN tar zxf /ws/aichallenge_submit.tar.gz -C /aichallenge/autoware/src

# Build
#RUN bash -c ' \
#  source /autoware/install/setup.bash; \
#  cd /aichallenge/autoware; \
#  rosdep update; \
#  rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO; \
#  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

#ENTRYPOINT []
#CMD ["bash", "/ws/main.bash"]
