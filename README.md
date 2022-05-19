# firmware-renesas-ck-ra6m5-internal
Edge Impulse enables developers to create the next generation of intelligent device solutions with embedded machine learning. This repository contains the Edge Impulse firmware for Renesas Cloud Kit RA6M5. This device supports all of Edge Impulse's device features, including ingestion, remote management and inferencing.

> **Note:** Do you just want to use this development board to collect data with Edge Impulse? No need to build this firmware. View the [getting started guide](https://docs.edgeimpulse.com/renesas/development-platforms/officially-supported-mcu-targets/renesas-ck-ra6m5) for a pre-built firmware image and flashing instructions. Or, you can use the [data forwarder](https://docs.edgeimpulse.com/renesas/edge-impulse-cli/cli-data-forwarder) to capture data from any sensor.

## Requirements

### Hardware

* CK-RA6M5

### Software
* [Renesas e2studio](https://www.renesas.com/us/en/software-tool/e-studio)
* [Renesas FSP 3.5.0](https://github.com/renesas/fsp/releases/tag/v3.5.0)
* [GNU Arm Embedded Toolchain 10.3.2021](https://developer.arm.com/downloads/-/gnu-rm)
* [Segger JLink software pack](https://www.segger.com/downloads/jlink)
* [Edge Impulse CLI](https://docs.edgeimpulse.com/docs/cli-installation)

Project created with e2studio v22.1.0 using FSP 3.5.0

Toolchain ARM GCC 10.3.2021

## How to build

### Using e2studio

> **Note:** e2studio is available for Windows 10 & 11 and Linux Os with x86_64 architecture.

1. Install [e2studio](https://www.renesas.com/us/en/software-tool/e-studio) (during installation add option for Renesas RA support) and download [FSP 3.5.0](https://github.com/renesas/fsp/releases/tag/v3.5.0)
1. Unpack FSP 3.5.0 in the directory ofe2studio looks for. It's OS depentand, can be checked from Help->CMSIS Packs Management->Renesas RA->Show in System Explorer. Go up to \internal folder and here the zip can be unpacked.
1. Verify that the toolchain is present checking in Help->Add Renesas Toolchains. If not, download the one for your OS and add it here.
1. Clone this repo and then import in e2studio: File->Import->General->Existing Projects into Workspace and then browse for the project.
1. To build the project: Project->Build All

### Build with docker

> **Note:** Docker build can be done with MacOs, Windows 10 & 11 and Linux with x86_64 architecture

1. Build container

    ```
    docker build -t edge-impulse-renesas .
    ```

1. Build firmware

    ```
    docker run --rm -v $PWD:/app/workspace/firmware-renesas-ck-ra6m5 edge-impulse-renesas
    ```

## Flashing
To flash the pre built image, run the script for your OS inside the /flash-script folder.
To flash the board, the JLink software package needs to be installed from here: https://www.segger.com/downloads/jlink and added to the PATH.

## Connect to the board
The connect to the board, a USB-to-serial converter is needed. 
The pin used are the TX and RX on the Arudino header (to be updated).
