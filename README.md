# Build Debian Packages for Autoware

The project builds Debian packages for Autoware in an isolated Docker
container environment.

## Prerequisites

- **Ubuntu 22.04** is recommended.
- **Docker**. You can follow the [instructions](https://docs.docker.com/engine/install/ubuntu/) to install Docker on Ubuntu.

## Usage

Prepare the Autoware repository in `~/autoware` directory with source
packages checked out in `~/autoware/src`.

```sh
# at home directory
git clone https://github.com/autowarefoundation/autoware.git

cd autoware
mkdir src
vcs import src < autoware.repos
```

> NOTE
>
> There is no need to run ansible scripts on the host system.

Download this project and place this project directory to
`~/autoware/rosdebian` directory. Now, launch the building script.

```sh
cd rosdebian
./start.sh
```

Once the build is done, find newly created Debian packages in
`~/autoware/release_deb`.

## Details

The build script runs the following steps.

1. Create a mirrored `src` directroy in
   `~/autoware/build_deb/_sources/src`.
2. Perform `rosdep install` to install required system dependencies.
3. Run `colcon build` to build binaries in
   `~/autoware/build_deb/_sources/install` and source the ROS environment.
4. Generate Debian control and rule files in `~/autoware/$pkg/debian`.
5. Build .deb files for all packages and move them to `~/autoware/release_deb`.
