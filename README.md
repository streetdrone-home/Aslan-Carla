# ASLAN-CARLA Integration

## Video Instructions

The installation and usage procedures are available as video at
(https://drive.google.com/drive/folders/1qunwCO7kN5JD7DAlgmE16ukNaXXPnIR9?usp=sharing)

## Introduction

Integration of CARLA (open-source autonomous driving simulator) with [Project ASLAN](https://www.project-aslan.org/) (open-source autonomous driving software for low-speed applications).

Project ASLAN resources are available at:
(https://github.com/project-aslan/Aslan/tree/melodic)

## Requirements

* Ubuntu 18.04 LTS
* ROS 1 Melodic [ros-melodic-desktop-full](http://wiki.ros.org/melodic/Installation/Ubuntu)
* Carla Aslan Mapper
* Carla 0.9.12
* Unreal Editor 4.26.2
* Aslan 1.x

To import a custom vehicle model:

* Blender 2.93.4

## Installation

### ROS

* Follow the official instructions to install ROS Melodic on Ubuntu:

    http://wiki.ros.org/melodic/Installation/Ubuntu

* Create a new workspace directory:

        mkdir -p aslan_carla_ws/src

### Carla & Unreal Engine

Follow the official instructions to install Carla on top of Unreal Engine:

https://carla.readthedocs.io/en/0.9.12/build_linux/

#### Notes

* The ASLAN-CARLA bridge currently requires CARLA version 0.9.12. Do the following when cloning the repository to ensure this version:

        git clone https://github.com/carla-simulator/carla.git -b 0.9.12

* In the “Build Carla” step, while compiling the Python API client, use the following command:

        make PythonAPI ARGS="--python-version=2.7,3.6"

* Install the wheel so that the ASLAN-CARLA bridge will be able to find it (the exact file names can differ based on versions of resources that you are using):

        pip3 install PythonAPI/carla/dist/carla-0.9.12-cp36-cp36m-linux_x86_64.whl  # For Python3
        pip install PythonAPI/carla/dist/carla-0.9.12-cp27-cp27mu-linux_x86_64.whl  # For Python2

### Aslan & Carla Aslan Bridge

* Clone the Project ASLAN and the bridge repositories into the workspace created before:

        cd aslan_carla_ws
        git clone --recurse-submodules https://github.com/project-aslan/Aslan.git src/Aslan
        git clone --recurse-submodules https://github.com/streetdrone-home/Aslan-Carla.git src/Aslan-Carla

* Still in the workspace, install dependencies:

        rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

* Compile the workspace using `catkin_make` (or `catkin build` if using catkin tools). This will compile both Aslan and the bridge together.

### Add Nissan e-NV200 model to CARLA

* Contact [Sander van Dijk](mailto:sander.vandijk@streetdrone.com) to obtain the model resources.

* Go to the `resources` directory of this repository:

        cd src/Aslan-Carla/src/resources

* Extract the model resources:

        unzip 'Nissan Env200.zip'

* Copy resources into CARLA (check the path to `carla` is correct in the commands below):

        cp -r 'Nissan Env200/Blueprint_Vehicles/Nissan_Env200LP' ~/carla/Unreal/CarlaUE4/Content/Carla/Blueprints/Vehicles/
        cp 'Nissan Env200/Blueprint_Vehicles/VehicleFactory.uasset' ~/carla/Unreal/CarlaUE4/Content/Carla/Blueprints/Vehicles/
        cp -r 'Nissan Env200/Static_Vehicles/Nissan_Env200LP' ~/carla/Unreal/CarlaUE4/Content/Carla/Static/Vehicles/4Wheeled/

## Usage

### Run Carla

* Open a terminal
* Navigate to the Carla root folder (e.g. `cd ~/carla`)
* Run Carla using:

        make launch

    After the first run you can also use `make launch-only` which skips some of the build steps.

* Once Carla is loaded, press the play button

### Run Carla-Aslan Bridge Module

* Open another terminal and navigate to the workspace (e.g. `cd ~/aslan_carla_ws`)
* Source the workspace:

        source devel/setup.bash

* Launch the bridge:

        roslaunch carla_aslan_mapper carla_aslan_spawn_vehicle.launch

    This launches the bridge plus other required nodes, as well as Rviz to monitor the vehicle. You can disable starting Rviz with:

        roslaunch carla_aslan_mapper carla_aslan_spawn_vehicle.launch rviz:=false

### Run Aslan

* Follow the instruction in the given link:
(https://github.com/project-aslan/Aslan)

* Before running any modules, make sure to go to the 'Simulation' tab and press the 'SimTime' button.

### Play demo scenario

This repository contains all resources needed to run a demo scenario, where the ego vehicle controlled by Aslan will drive straight along a road, when another vehicle will come from behind, change lane to pass and then change lane back to cut in in front of the ego vehicle, forcing the ego vehicle to slow down, before driving off again.

Firstly, download and setup the CARLA scenario runner:
* Download [the source code asset for version 0.9.12](https://github.com/carla-simulator/scenario_runner/releases/tag/v0.9.12) and unpack it into the `aslan_carla_ws` directory, so that it is at `aslan_carla_ws/scenario_runner-0.9.12/`.
* Install its requirements with:

        pip3 install -r scenario_runner-0.9.12/requirements.txt

    Also ensure the following requirements are installed:

        pip3 install transforms3d
        sudo apt install python3-pexpect

* Export the following environment variables:

        export CARLA_ROOT=${HOME}/carla
        export PYTHONPATH=${PYTHONPATH}:${CARLA_ROOT}/PythonAPI/carla

    Ensure that the path to `CARLA_ROOT` matches your system. You can add these exports to your `~/.profile` file to prevent having to run them anew in each terminal.

After that, you are ready to run the scenario:

* Run Carla with `make launch-only` and press Play.
* Source the workspace using `source devel/setup.bash`.
* Spawn the vehicle with `roslaunch carla_aslan_mapper carla_aslan_spawn_vehicle.launch`.
* Optionally, in the Rviz instance that has opened, load Aslan's configuration by going to File -> Open Config, then browse to `aslan_carla_ws/src/Aslan/src/aslan_tools/rviz/default.rviz`, to get a better insight into what Aslan is doing during the scenario.
* Run the scenario with:

        python3 scenario_runner-0.9.12/scenario_runner.py --openscenario $(rospack find carla_aslan_mapper)/scenarios/Overtake.xosc --waitForEgo

Fly to the crossing just infront of the footbridge in CARLA (forward and to the right from the start position of the camera) to find the two vehicles that will play out rhe overtake scenario. You can also see it unfold in Rviz.
