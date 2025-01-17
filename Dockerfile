FROM livo:ros2 AS development
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
FROM development AS deploy
# TODO: install production "runner" package and webapp
