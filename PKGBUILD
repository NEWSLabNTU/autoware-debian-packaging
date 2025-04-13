# Maintainer: Lin, Hsiang-Jui <jerry73204@gmail.com>
pkgname=pack-rosdeb
pkgver=0.1.0
pkgrel=1
pkgdesc='Create Debian packages for a ROS2 colcon repository'
arch=('all')
builddepends=('rsync')
depends=('python3-rocker' 'rsync' 'parallel' 'ros-dev-tools')
license=('MIT')

source=(
    'pack-rosdeb.tar'
    'pack-rosdeb.sh'
)
sha256sums=(
    'SKIP'
    '01101714a094c94253b1ed1bbc40e6d56d101be21b7799b1e44219ea41585c48'
)

package() {
    install -dm755 "$pkgdir/usr/lib/pack-rosdeb"
    rsync -aP "$srcdir/pack-rosdeb/" "$pkgdir/usr/lib/pack-rosdeb"
    
    install -dm755 "$pkgdir/usr/bin"
    install -Dm755 "$srcdir/pack-rosdeb.sh" "$pkgdir/usr/bin/pack-rosdeb"
}

# vim: set sw=4 expandtab:
