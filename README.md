# Airdropping Sensor Networks from Drones and Insects

## Abstract
We present the first system that can airdrop wireless sensors from small drones and live insects. In addition to the challenges of achieving low-power consumption and long-range communication, deploying wireless sensors is difficult because it requires the sensor to survive the impact when dropped in mid-air. Our design takes inspiration from nature: small insects like ants can fall from tall buildings and survive because of their tiny mass and size. Inspired by this, we design insect-scale wireless sensors that come fully integrated with an onboard power supply and a lightweight mechanical actuator to detach from the aerial platform. Our system introduces a first-of-its-kind 37 mg mechanical release mechanism to drop the sensor during flight, using only 450 μJ of energy as well as a wireless communication link that can transmit sensor data at 33 kbps up to 1 km. Once deployed, our 98 mg wireless sensor can run for 1.3-2.5 years when transmitting 10-50 packets per hour on a 68 mg battery. We demonstrate attachment to a small 28 mm wide drone and a moth (Manduca sexta) and show that our insect-scale sensors flutter as they fall, suffering no damage on impact onto a tile floor from heights of 22 m.

## Setup
1. Download nRF SDK
-  Go to https://www.nordicsemi.com/Software-and-Tools/Software/nRF5-SDK/Download#infotabs
-  Select Version 15.3.0 and download it
-  Unzip the .zip to a high level directory (e.g. C:\ on Windows, or home directory on linux/mac)
-  Update GNU_INSTALL_ROOT (path of your gcc) inside Makefile.posix. Details can be found here: https://aaroneiche.com/2016/06/01/programming-an-nrf52-on-a-mac/
2. Clone the repo
-  Clone in this location: path/to/zip/SDK/examples/ble_peripheral
3. Build and flash
-  CD into pca10040/s132/armgcc
-  Run 'make flash' in terminal to build and flash 

## Branches
This repository includes three branches:
-  The 'master'branch is a starting point for any project that wants to incorporate the channel hopping and de-whitening code into a new project. 
-  'mk/packet-error-rate-test' contains code which will indefinitely broadcast beacons, useful for assessing link budget and other desktop performance tests
-  'mk/deploy' contains an example of a full system demo. After programming, the BLE microcontroller will listen for a packet from the base-station, and self-deploy once the appropriate packet has been received. After deployment, the microcontroller will begin sending beacons at 33kbps to maximize the link budget.
