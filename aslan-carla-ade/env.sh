#!/usr/bin/env bash

source "/opt/ros/$ROS_DISTRO/setup.bash"

for x in /opt/*; do
    if [[ -e "$x/.env.sh" ]]; then
	source "$x/.env.sh"
    fi
done

cd
