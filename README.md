# Airdropping Sensor Networks from Drones and Insects

## Abstract
We present the first system that can airdrop wireless sensors from small drones and live insects. In addition to the challenges of achieving low-power consumption and long-range communication, deploying wireless sensors is difficult because it requires the sensor to survive the impact when dropped in mid-air. Our design takes inspiration from nature: small insects like ants can fall from tall buildings and survive because of their tiny mass and size. Inspired by this, we design insect-scale wireless sensors that come fully integrated with an onboard power supply and a lightweight mechanical actuator to detach from the aerial platform. Our system introduces a first-of-its-kind 37 mg mechanical release mechanism to drop the sensor during flight, using only 450 μJ of energy as well as a wireless communication link that can transmit sensor data at 33 kbps up to 1 km. Once deployed, our 98 mg wireless sensor can run for 1.3-2.5 years when transmitting 10-50 packets per hour on a 68 mg battery. We demonstrate attachment to a small 28 mm wide drone and a moth (Manduca sexta) and show that our insect-scale sensors flutter as they fall, suffering no damage on impact onto a tile floor from heights of 22 m.

## Setup
1. Download nRF SDK
-  Go to https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download#infotabs
-  Download the latest SDK
-  Unzip the .zip to a high level directory (e.g. C:\ on Windows, or home directory on linux/mac)
2. Install toolchains
-  Install gcc-arm-none-eabi using 'brew install gcc-arm-none-eabi'
-  Update GNU_INSTALL_ROOT (path of your gcc) inside Makefile.posix of the unzipped SDK. If you installed using brew, the path should be /usr/local/Cellar/gcc-arm-none-eabi/20180627/bin/
-  Install nordic command line tools from here: https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Command-Line-Tools/Download. Unzip, and move the nrfjprog directory to /usr/local/. Finally, add this to your PATH inside .bash_profile with this change: export PATH=/usr/local/bin:/usr/local/sbin:/usr/local/nrfjprog:$PATH
-  Validate that this worked by opening a new terminal and typing 'nrfjprog --version' as a sanity check
3. Clone the repo
-  Clone in this location: path/to/zip/SDK/examples/ble_peripheral
4. Build and flash
-  CD into pca10040/s132/armgcc
-  Run 'make flash' in terminal to build and flash

If you have any trouble installing, feel free to reach out, more details for setting up on a mac can be found here: https://aaroneiche.com/2016/06/01/programming-an-nrf52-on-a-mac/

## Branches
This repository includes three branches:
-  The 'master'branch is a starting point for any project that wants to incorporate the channel hopping and de-whitening code into a new project.
-  'mk/packet-error-rate-test' contains code which will indefinitely broadcast beacons, useful for assessing link budget and other desktop performance tests
-  'mk/deploy' contains an example of a full system demo. After programming, the BLE microcontroller will listen for a packet from the base-station, and self-deploy once the appropriate packet has been received. After deployment, the microcontroller will begin sending beacons at 33kbps to maximize the link budget.

## Receiver
1. Record data from SDR using GNURadio. (frequency - 2.402GHz, sampling rate - 2MHz)
2. Process .dat file in MATLAB using provided LongRangeBLE_noisy_decoding.m
- function [ packets, filtered ] = LongRangeBLE_noisy_decoding(filename, frequencyDivider, miniarg)
- packets: raw packets, filtered: packets remaining after filtering, filename:path to .dat file, frequencyDivider: Amount of bit stretching done on transmission side, miniarg: string which corresponds to path of preamble.dat file.
3. NOTE: The preamble for your specific transmitter needs to be recorded manually using the USRP for best results. One can record transmissions and extract a preamble manually in MATLAB.
