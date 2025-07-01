#!/bin/bash
docker build --network=host --target development -t livo:dev .
