#!/usr/bin/env bash
# Maintainer: Autoware Foundation <contact@autoware.org>

pkgname=colcon2deb
pkgver=0.1.0
pkgrel=1
pkgdesc="Build Debian packages from colcon workspaces"
arch=('all')
url="https://github.com/autowarefoundation/colcon2deb"
license=('Apache-2.0')

depends=(
    'python3'
    'python3-yaml'
    'docker-ce'
)

makedepends=(
    'python3'
)

source=("colcon2deb-${pkgver}.tar.gz")
sha256sums=('SKIP')

package() {
    # Create directories
    install -dm755 "${pkgdir}/usr/bin"
    install -dm755 "${pkgdir}/usr/share/${pkgname}/helper"
    install -dm755 "${pkgdir}/usr/share/${pkgname}/example"
    install -dm755 "${pkgdir}/usr/share/doc/${pkgname}"

    cd "${srcdir}/colcon2deb-${pkgver}"

    # Install main executable
    install -Dm755 "colcon2deb" "${pkgdir}/usr/bin/colcon2deb"

    # Install helper scripts
    cp -r "helper/"* "${pkgdir}/usr/share/${pkgname}/helper/"
    chmod 755 "${pkgdir}/usr/share/${pkgname}/helper/"*.sh

    # Install example files (excluding docker/ subdirectory)
    cp -r "example/config" "${pkgdir}/usr/share/${pkgname}/example/"
    install -Dm644 "example/config.yaml" "${pkgdir}/usr/share/${pkgname}/example/config.yaml"
    
    # Copy the example Dockerfile symlink if it exists
    if [ -L "example/Dockerfile" ]; then
        # Get the relative path of the symlink target
        local link_target=$(readlink "example/Dockerfile")
        # Create a note about the Dockerfile
        cat > "${pkgdir}/usr/share/${pkgname}/example/Dockerfile.README" << EOF
The example Dockerfile is a symlink to: ${link_target}

To use it, you'll need to obtain the actual Dockerfile from the colcon2deb repository.
EOF
    fi

    # Install documentation
    cat > "${pkgdir}/usr/share/doc/${pkgname}/README.md" << 'EOF'
# colcon2deb

Build Debian packages from colcon workspaces using Docker containers.

## Usage

```bash
colcon2deb --workspace /path/to/workspace --config /path/to/config.yaml
```

## Configuration

Create a config.yaml file with the following structure:

```yaml
version: 1

docker:
  # Option 1: Use existing image
  image: ros:humble-ros-base
  
  # Option 2: Build from Dockerfile
  # dockerfile: ./Dockerfile

output:
  directory: ./output

packages:
  directory: ./config  # Directory with package-specific debian/ folders

build:
  ros_distro: humble
  parallel_jobs: 8
  skip_tests: true
```

## Example

See example configuration and package configs in `/usr/share/colcon2deb/example/`

## Helper Scripts

The build process uses helper scripts installed in `/usr/share/colcon2deb/helper/`
EOF

    # Create a simple changelog
    cat > "${pkgdir}/usr/share/doc/${pkgname}/changelog" << EOF
colcon2deb (${pkgver}-${pkgrel}) unstable; urgency=low

  * Initial release

 -- Autoware Foundation <contact@autoware.org>  $(date -R)
EOF
    gzip -9 "${pkgdir}/usr/share/doc/${pkgname}/changelog"
}

# vim: set ts=4 sw=4 et:
