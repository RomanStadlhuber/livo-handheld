#!/bin/bash
docker build -f Dockerfile.mapping --network=host -t livo-mapping:dev .
