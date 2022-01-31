#!/bin/bash

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
ROOT="$(cd $DIR/../ && pwd)"

# NOTE: this is used in a docker build, so do not run any scripts here.

function install_git_lfs_linux_arm64() {
  pushd /tmp
  wget https://github.com/git-lfs/git-lfs/releases/download/v3.0.2/git-lfs-linux-arm64-v3.0.2.tar.gz
  tar -xvf git-lfs-linux-arm64-v3.0.2.tar.gz
  ./install.sh
  popd
}

function install_toolchain_xenial() {
  sudo apt-get install -y clang-8 software-properties-common
  sudo apt-get update
  sudo rm /usr/bin/clang /usr/bin/clang++
  sudo ln -s /usr/bin/clang-8 /usr/bin/clang
  sudo ln -s /usr/bin/clang++-8 /usr/bin/clang++

  sudo apt-get install -y g++-8
  sudo rm /usr/bin/gcc /usr/bin/g++
  sudo ln -s /usr/bin/gcc-8 /usr/bin/gcc
  sudo ln -s /usr/bin/g++-8 /usr/bin/g++
}

function install_capnp() {
  CAPNP_VERSION=0.8.0

  pushd /tmp
  curl -O https://capnproto.org/capnproto-c++-$CAPNP_VERSION.tar.gz
  tar zxf capnproto-c++-$CAPNP_VERSION.tar.gz

  pushd capnproto-c++-$CAPNP_VERSION
  ./configure
  make -j7 check
  sudo make install
  popd

  popd
}

function install_libi2c() {
  # we just skip i2c ifdef
  sudo apt-get install -y libi2c-dev  # don't think it works
  sudo apt-get install -y linux-headers-4.4.0-1156-snapdragon  # don't think it works
}

function install_libgsl() {
  # /usr/lib/aarch64-linux-gnu/libgsl.so
  # /usr/lib/aarch64-linux-gnu/libgsl.so.19
  # /usr/lib/aarch64-linux-gnu/libgsl.so.19.0.0

  # sudo apt-file search libgsl.so
  sudo apt-get install -y libgsl-dev  # libgsl2
}

function install_libcb() {
  # /usr/lib/aarch64-linux-gnu/libCB.so

  # sudo apt-file search libCB.so
  # sudo apt-get install -y libCB.so
  sudo ln -s /android/vendor/lib64/libCB.so /usr/lib/aarch64-linux-gnu/libCB.so
}

function install_ffmpeg() {
  pushd /tmp
  wget https://ffmpeg.org/releases/ffmpeg-4.2.2.tar.bz2
  tar xvf ffmpeg-4.2.2.tar.bz2

  pushd ffmpeg-4.2.2
  ./configure --enable-shared --disable-static
  make -j$(nproc)
  sudo make install
  popd

  popd
}

function install_omxcore() {
  # /usr/lib/aarch64-linux-gnu/libOmxCore.so
  # sudo ln -s /android/vendor/lib64/libOmxCore.so /usr/lib/aarch64-linux-gnu/libOmxCore.so
}

function install_qtbase5_private() {
  # qpa/qplatformnativeinterface.h
  sudo apt-get install -y qtbase5-private-dev
}

# cmake 3.10 (or above) required for building mapbox-gl-native
function build_cmake() {
  pushd /tmp
  wget https://cmake.org/files/v3.14/cmake-3.14.4.zip
  unzip cmake-3.14.4.zip

  pushd cmake-3.14.4
  cmake .
  make -j$(nproc)
  # sudo make install
  popd

  popd
}

function build_mapbox() {
  build_cmake()

  sudo apt install -y libqt5opengl5-dev

  pushd /tmp
  git clone --recursive https://github.com/commaai/mapbox-gl-native.git

  pushd mapbox-gl-native
  mkdir build

  pushd build
  /tmp/cmake-3.14.4/bin/cmake -DMBGL_WITH_QT=ON ..
  make -j$(nproc) mbgl-qt
  # sudo cp libqmapboxql.so /usr/local/lib
  popd

  popd

  popd
}

# Install packages present in all supported versions of Ubuntu
function install_ubuntu_common_requirements() {
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends \
    autoconf \
    build-essential \
    ca-certificates \
    clang \
    cmake \
    make \
    cppcheck \
    libtool \
    gcc-arm-none-eabi \
    bzip2 \
    liblzma-dev \
    libarchive-dev \
    libbz2-dev \
    capnproto \
    libcapnp-dev \
    curl \
    libcurl4-openssl-dev \
    git \
    # ffmpeg \
    libavformat-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavutil-dev \
    libavfilter-dev \
    libeigen3-dev \
    libffi-dev \
    libglew-dev \
    libgles2-mesa-dev \
    libglfw3-dev \
    libglib2.0-0 \
    libomp-dev \
    libopencv-dev \
    libpng16-16 \
    libssl-dev \
    libsqlite3-dev \
    libusb-1.0-0-dev \
    libzmq3-dev \
    libsystemd-dev \
    locales \
    opencl-headers \
    ocl-icd-libopencl1 \
    ocl-icd-opencl-dev \
    clinfo \
    python-dev \
    qml-module-qtquick2 \
    qtmultimedia5-dev \
    qtlocation5-dev \
    qtpositioning5-dev \
    libqt5sql5-sqlite \
    libqt5svg5-dev \
    libqt5x11extras5-dev \
    libreadline-dev \
    libdw1 \
    valgrind
}

# Install Ubuntu 21.10 packages
function install_ubuntu_latest_requirements() {
  install_ubuntu_common_requirements

  sudo apt-get install -y --no-install-recommends \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools
}

# Install Ubuntu 20.04 packages
function install_ubuntu_lts_requirements() {
  install_ubuntu_common_requirements

  sudo apt-get install -y --no-install-recommends \
    libavresample-dev \
    qt5-default
}

# Detect OS using /etc/os-release file
if [ -f "/etc/os-release" ]; then
  source /etc/os-release
  case "$ID $VERSION_ID" in
    "ubuntu 21.10")
      install_ubuntu_latest_requirements
      ;;
    "ubuntu 20.04")
      install_ubuntu_lts_requirements
      ;;
    *)
      echo "$ID $VERSION_ID is unsupported. This setup script is written for Ubuntu 20.04."
      read -p "Would you like to attempt installation anyway? " -n 1 -r
      echo ""
      if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
      fi
      install_ubuntu_lts_requirements
  esac
else
  echo "No /etc/os-release in the system"
  exit 1
fi


# install python dependencies
$ROOT/update_requirements.sh

source ~/.bashrc
if [ -z "$OPENPILOT_ENV" ]; then
  printf "\nsource %s/tools/openpilot_env.sh" "$ROOT" >> ~/.bashrc
  source ~/.bashrc
  echo "added openpilot_env to bashrc"
fi

echo
echo "----   OPENPILOT SETUP DONE   ----"
echo "Open a new shell or configure your active shell env by running:"
echo "source ~/.bashrc"
