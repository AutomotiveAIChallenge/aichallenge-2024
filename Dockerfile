FROM osrf/ros:humble-desktop AS common

RUN apt-get update
RUN apt-get -y install libgl1-mesa-glx libgl1-mesa-dri
RUN apt-get -y install iproute2

FROM common AS dev

ENV ROS_LOCALHOST_ONLY 1
ENV RCUTILS_COLORIZED_OUTPUT 1

FROM common AS eval

RUN mkdir /ws
RUN git clone --depth 1 https://github.com/AutomotiveAIChallenge/aichallenge-source-materials /ws/repository
RUN mv /ws/repository/aichallenge /aichallenge
RUN rm -rf /aichallenge/simulator
RUN rm -rf /aichallenge/autoware/src/aichallenge_submit
RUN chmod 757 /aichallenge

COPY aichallenge/simulator/ /aichallenge/simulator/
COPY output/aichallenge_submit.tar.gz /ws
RUN tar zxf /ws/aichallenge_submit.tar.gz -C /aichallenge/autoware/src

RUN bash -c ' \
  source /autoware/install/setup.bash; \
  cd /aichallenge/autoware; \
  rosdep update; \
  rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO; \
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

# ENTRYPOINT []
# CMD ["bash", "/ws/main.bash"]
