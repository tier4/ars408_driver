FROM ros:foxy
WORKDIR /app
RUN git clone --branch ros2 https://gitlab.com/perceptionengine/autoware/autoware_iv_msgs.git /app/src/iv_msgs/
RUN git clone https://github.com/autowarefoundation/ros2_socketcan.git /app/src/can/
COPY ./* /app/src/ars408/
# COPY ./include/* /app/src/ars408/include/
# COPY ./launch/* /app/src/ars408/launch/
# COPY ./include/* /app/src/ars408/include/
RUN apt-get update && apt-get install libboost1.71-dev
RUN rosdep update
RUN rosdep install -y --from-paths src --ignore-src
RUN /bin/bash -c 'source /opt/ros/foxy/setup.bash && colcon build'
