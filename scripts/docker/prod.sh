#! /bin/bash
docker build --target deploy -t livo:deploy .
# remove container if it already exists
docker container rm --force livo-prod
# NOTE: restart policy needs to be "always"
# otherwise, restart will not take effect after reboot...
# for more information, see:
# https://docs.docker.com/config/containers/start-containers-automatically/
docker run \
--privileged \
--network host \
-v /dev:/dev \
--name livo-prod \
--restart always \
livo:deploy
