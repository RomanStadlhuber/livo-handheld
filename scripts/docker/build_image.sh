#!/bin/bash
# build only up to and including development stage
docker build --target development -t livo .
