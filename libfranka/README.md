# libfranka: C++ library for Franka Emika research robots

With this library, you can control research versions of Franka Emika robots.

# Fork Information

This fork is supposed to be the latest version of libfranka that works with the franka panda arm version 3.0.3. It has a few changes made to it to compile on Ubuntu 24.04 LTS and work with ROS2 Jazzy.

# Installation and Build
Clone the repository and submodules:
```bash
git clone https://github.com/alexmnr/libfranka.git
cd libfranka
git submodule update --init --recursive
```

Install Dependencies:
```bash
sudo apt-get install -y build-essential cmake git libpoco-dev  libfmt-dev
```

Build it:
```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_STANDARD=14 \
      -DBUILD_TESTS=OFF \
      -DBUILD_EXAMPLES=ON ..
make
sudo make install
sudo ldconfig
```
