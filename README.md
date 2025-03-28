# Build Debian Packages for Autoware

The project builds Debian packages for Autoware in an isolated Docker
container environment.

## Prerequisites

- **Ubuntu 22.04** operating system is recommended.
- **Docker**. You can follow the [instructions](https://docs.docker.com/engine/install/ubuntu/) to install Docker on Ubuntu.

## Usage

### Prepare Autoware Source Repository

Prepare the Autoware version 2024.11 repository in the `~/autoware`
directory. It is recommended to use the patched Autoware repository
from NEWSLab.

<details>
<summary>Option 1: Use Patched Autoware from NEWSLab (Recommended)</summary>

```sh
git clone -b rosdebian/2024.11 --recurse-submodules https://github.com/NEWSLabNTU/autoware.git
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
autoware-localrepo_1.0-1_amd64.deb
```

### Install Autoware Debian Packages

Copy the `autoware-localrepo_1.0-1_amd64.deb` to the target system
you'd like to deploy. Run the commands below to install Autoware.

```sh
sudo dpkg -i autoware-localrepo_1.0-1_amd64.deb
sudo apt update
sudo apt install ros-humble-autoware-full
```

## Details

The build script runs the following steps.

1. Create a mirrored `src` directroy in
   `~/autoware/build_deb/sources/src`.
2. Perform `rosdep install` to install required system dependencies.
3. Run `colcon build` to build binaries in
   `~/autoware/build_deb/sources/install` and source the ROS environment.
4. Prepare Debian control and rule files in
   `~/autoware/build_deb/build/$pkg/debian`. These files are either
   copied from `~/autoware/rosdebian/config/$pkg/debian` or are
   generated on the fly.
5. Build Debian packages for all packages and move them to
   `~/autoware/build_deb/dist`.

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
