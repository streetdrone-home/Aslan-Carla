#!/bin/bash
carla_path=~/carla
aslan_path=~/Aslan/Aslan
traffic_path=~/carla/PythonAPI/examples/
no_of_vehicles=100

echo "Starting Carla, Aslan and Bridge ... "
bash -c "(cd ${carla_path} && make launch) & (cd ${aslan_path} && ./run) & (sleep 180 && roslaunch carla_aslan_mapper carla_ros_custom_vehicle.launch") && cd ${traffic_path} && python carla/PythonAPI/examples/generate_traffic.py -n ${no_of_vehicles}


