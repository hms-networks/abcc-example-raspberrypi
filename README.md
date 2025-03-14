# abcc-example-raspberrypi
This is a stand-alone example application using the Anybus CompactCom Driver API ([abcc-api](https://github.com/hms-networks/abcc-driver-api)) ported for the [Anybus CompactCom Adapter Board - Module to Raspberry Pi](https://www.hms-networks.com/p/029860-b-anybus-compactcom-adapter-board-module-to-raspberry-pi) evaluation platform.
### Purpose
To enable easy evaluation and inspiration to [Anybus CompactCom](https://www.hms-networks.com/embedded-network-interfaces) prospects.

## Prerequisites
### System
- This example application shall be built for and ran in a Linux environment.
### LIBGPIOD
- The library LIBGPIOD is required. Install this library with the following command:
```
apt install libgpiod-dev
```
### CMake
- If you do not yet have CMake, [download](https://cmake.org/download/) and install it before continuing.

## Cloning
### Flag? What flag?
This repository contain submodules [abcc-driver-api](https://github.com/hms-networks/abcc-api), ([abcc-driver](https://github.com/hms-networks/abcc-driver) and [abcc-abp](https://github.com/hms-networks/abcc-abp) that must be initialized. Therefore, pass the flag `--recurse-submodules` when cloning.

```
git clone --recurse-submodules https://github.com/hms-networks/abcc-example-raspberrypi.git
```
#### (In case you missed it...)
If you did not pass the flag `--recurse-submodules` when cloning, the following command can be run:
```
git submodule update --init --recursive
```

## Build and run
### CMake
This example application utilizes the Anybus CompactCom Driver API's CMake module file in a top level CMakelLists.txt file to generate a complete Visual Studio project. To generate the project, run the lines below, starting in the repository root.
```
mkdir build
```
```
cd build
```
```
cmake ..
```

### Make
Run the generated *Makefile* file to compile.
```
make
```
### Run
Run the generated executable.
```
./main
```
#### (Nothing is happening...)
Make sure that SPI is enabled in your Raspberry Pi configurations and that your CompactCom is attached correctly. See the [Adapter Board for Raspberry Pi INSTALLATION GUIDE](https://hmsnetworks.blob.core.windows.net/nlw/docs/default-source/products/anybus/manuals-and-guides---manuals/hms-scm-1202-225.pdf?sfvrsn=8c728ed7_4) for more details.

