# Aslan-Carla ADE environment

The files in this directory provide the configuration to set up a development environment to work with the Aslan-Carla bridge, that can be used with [ADE]((https://ade-cli.readthedocs.io/en/latest/index.html).

## Usage

### Install Docker

Make sure you have the latest version of Docker installed by following the official installation instructions:

* [Ubuntu](https://docs.docker.com/engine/install/ubuntu/)

To be able to use NVIDIA GPUs, also install the [NVIDIA Docker Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

### Install ADE

Follow the [documentation of ADE](https://ade-cli.readthedocs.io/en/latest/index.html) to install the `ade` command line tool.

### Build Docker image

Run the following to build the Docker image:
```
cd ade-aslan
docker build -t aslan-carla-ade .
```
If this fails due to permissions, do *not* run with `sudo`.
Instead, go back to the Docker installation instructions and perform the step to add yoruself to the `docker` group which you have missed.

### Create a home

When running ADE it will be like logging into a different machine with a separate home directory.
This directory is actually a directory on your machine that gets mounted inside of the container, so you can easily share files in and outside of the environment.
You need to create this folder and mark it as the home folder for ADE:
```
mkdir adehome
cd adehome
touch .adehome
```

Next you need to specify what images and volumes ADE should use when starting it inside of this home directory. Copy the `.aderc` file from this directory into `cavlhome` to do so.

### Start the environment

Now you should be ready to start and enter the environment by running the following inside the `cavlhome` directory:
```
ade start
ade enter
```

