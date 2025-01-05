#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

echo "$0: Adding MRS Stable PPA repository"

apt-get -y install curl gpg dpkg-dev

ARCH=$(dpkg-architecture -qDEB_HOST_ARCH)

curl -s --compressed "https://ctu-mrs.github.io/ppa-stable/ctu-mrs.gpg" | gpg --dearmor |  tee /etc/apt/trusted.gpg.d/ctu-mrs.gpg >/dev/null
curl -s --compressed -o /etc/apt/sources.list.d/ctu-mrs-stable.list "https://ctu-mrs.github.io/ppa-stable/ctu-mrs-apt.list"
curl -s --compressed -o /etc/apt/preferences.d/ctu-mrs-stable-preferences "https://ctu-mrs.github.io/ppa-stable/ctu-mrs-ppa-preferences.txt"
curl -s --compressed -o /etc/ros/rosdep/sources.list.d/ctu-mrs-stable.list "https://ctu-mrs.github.io/ppa-stable/ctu-mrs-$ARCH.list"
apt-get -y update

# rosdep update

echo "$0: Finished adding MRS Stable PPA repository"
