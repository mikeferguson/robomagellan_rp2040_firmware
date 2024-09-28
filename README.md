# robomagellan_rp2040_firmware

Firmware for my RP2040 RoboMagellan controller

## Setup

Setup SDK:

```
cd ~/bin
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

In this package

```
export PICO_SDK_PATH=~/bin/pico-sdk
git submodule update --init
mkdir build
cd build
cmake ..
make
```

# Known Issues

 * Deadman radio is not implemented
