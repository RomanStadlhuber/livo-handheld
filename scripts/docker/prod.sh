#! /bin/bash
docker build --target deploy -t livo:deploy .
docker run \
--privileged \
--network host \
-v /dev:/dev \
--name livo-prod \
--restart unless-stopped \
livo:deploy