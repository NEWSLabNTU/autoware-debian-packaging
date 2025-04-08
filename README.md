# Build Debian Packages for Autoware

The project builds a local Debian repository for Autoware in an
isolated Docker container environment.

The article gives instructions to build a "local repo" package. If
you're looking for the instructions to install Autoware from the local
repo, please skip to the *Install Autoware from the Local Debian
Repository* section.

## Prerequisites

- **Ubuntu 22.04** operating system is recommended.
- **Docker**. You can follow the
  [instructions](https://docs.docker.com/engine/install/ubuntu/) to
  install Docker on Ubuntu.

## Build the Local Debian Repository

### Prepare Autoware Source Repository

Prepare the Autoware repository in the `~/autoware` directory. It is
recommended to use the patched Autoware repository from NEWSLab.

<details>
<summary>Option 1: Use Patched Autoware from NEWSLab (Recommended)</summary>
```sh
git clone -b rosdebian/2025.02 --recurse-submodules https://github.com/NEWSLabNTU/autoware.git
cd autoware
```
</details>

<details>
<summary>Option 2: Use Official Autoware</summary>

```sh
# at home directory
git clone https://github.com/autowarefoundation/autoware.git

cd autoware
mkdir src
vcs import src < autoware.repos
```
</details>

> NOTE
>
> There is no need to run ansible scripts on the host system.

### Build Debian Packages

Download this project and place this project directory to
`~/autoware/rosdebian` directory.

```sh
# Within the `autoware` directory
git clone https://github.com/NEWSLabNTU/autoware-debian-packaging.git rosdebian
cd rosdebian
```

Launch the build process. It will place all artifacts in the
`~/autoware/build_deb` directory. Once the build is done, newly
compiled Debian packages can be found in `~/autoware/build_deb/dist`.

```sh
./start.sh
```

After the script finishes, a _repository_ package is create along side
the `start.sh`.

```
# The name may vary depending on your system.
autoware-localrepo_2025.2-1_amd64.deb
```

## Install Autoware from the Local Debian Repository

Copy the `autoware-localrepo_2025.2-1_amd64.deb` to the target system
and install the local repository.

```sh
sudo dpkg -i autoware-localrepo_2025.2-1_amd64.deb
sudo apt update
sudo apt install autoware-full
```

Configure the system. The step can be executed only once after
installation.

```sh
sudo autoware-setup
```


Run the [planning-simulation
tutorial](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/)
for example. Download required map and data files. Source
`/opt/autoware/autoware-env` to enable the runtime environment and
launch to example.

```sh
# Assume all map and artifacts are downloaded.
source /opt/autoware/autoware-env
ros2 launch autoware_launch planning_simulator.launch.xml \
	map_path:=$HOME/autoware_map/sample-map-planning \
	vehicle_model:=sample_vehicle \
	sensor_model:=sample_sensor_kit
```

## Details

The build script runs the following steps.

1. Create a cloned `src` directroy in
   `~/autoware/build_deb/sources/src`.
2. Run `rosdep install` to install required system dependencies.
   Additional dependencies are installed as well.
3. Run `colcon build` to build binaries in
   `~/autoware/build_deb/sources/install` and source its `setup.bash`.
4. Prepare Debian packaging script
   `~/autoware/build_deb/build/$pkg/debian`. These directories are
   either copied from `~/autoware/rosdebian/config/$pkg/debian` if it
   exists, or are generated on the fly using `bloom-generate`.
5. Build Debian packages for all packages in
   `~/autoware/build_deb/dist`.
6. Create additional Debian packages.
  - `autoware-runtime`, a virtual package that pulls all ROS packages from Autoware.
  - `autoware-config`, providing RMW and environment configuration scripts.
  - `autoware-full`, which pulls `autoware-runtime` and
    `autoware-config` packages for end users.
7. Create a local Debian repository and pack them into an
   `autoware-localrepo` package. It contains all Debian files in all
   previous steps.

## Customization

### Change Base Docker Image

The base image is specified in the first line of Dockerfile. You can
modify it to your preferred image. For example,

```diff
< FROM ubuntu:22.04 AS base
---
> FROM nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel AS base
```

### Customize Environment Setup

By the time the Docker image is built, the `setup/` directory is
copied to the `/workspace/setup` in the container and
`/workspace/setup/setup.sh` is executed. The setup script can be
modified to fit your needs.
