FROM livo:ros2 AS development
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
# copy to a global path so it is always available in both dev and deploy
COPY recorder_runner/launch/MID360_config.json /MID360_config.json
WORKDIR /ros2_ws
FROM development AS deploy
COPY recorder_runner /ros2_ws/src/recorder_runner
# build recorder_runner
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash &&  \
    source /ros2_ws/install/setup.bash && \
    source /ros2_ws/src/recorder_runner/build.sh"
COPY webinterface /app
WORKDIR /app
# instal webapp dependencies
RUN pip3 install --no-cache-dir -r requirements.txt
# run the webapp
COPY scripts/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]