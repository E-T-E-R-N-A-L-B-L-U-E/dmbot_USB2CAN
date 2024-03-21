# Linux Driver for DMBot USB2CAN Device

## Overview

    This project realized linux driver for DMBot USB2CAN Device.

    |        Function       | Realized or not |
    | --------------------- | --------------- |
    |  Send can data frame  |       Y         |
    |  Send can remote frame |      N         |
    |     Send UART frame   |       Y         |
    |  Recv can data frame  |       Y         |
    |   connection check    |       N         |
    |   usb2can setting     |       Y         |
    |    canbus setting     |       Y         |
    |   device reconnect    |       N         |
    |  Send can failure handle |    N         |


## Compile

This project only depends on basic cxx compile environment.

You can build this project with cmake.

```bash
mkdir build
cd build
cmake ..
make -j3
```