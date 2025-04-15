# GRiSP2 Robot

## Introduction

This project is an extension of the master thesis by François Goens and Cédric Ponsard, titled [*Dynamic balancing in the real world with GRiSP*](http://hdl.handle.net/2078.1/thesis:48907). It is developed in the context of a new master thesis by **Arthur Dandoy** and **Sam Raymakers**.

The initial work focused on designing a dynamically balanced, self-righting two-wheeled robot using a GRiSP2 board and the Hera sensor fusion framework.  
Our objective is to repurpose and extend this system into a **smart, motorized piece of everyday furniture**, specifically a **table**, that can:

- **move autonomously within a room**  
- maintain **static and dynamic balance**  
- implement an **obstacle avoidance mechanism**  

The architecture is based on **Erlang/OTP (version 27.2.4)** for its real-time concurrency model and fault-tolerance. It runs on **GRiSP2 boards** (developed by [Stritzinger GmbH](https://www.grisp.org/)), and interfaces with:

- **Pmod NAV**: for dimensional orientation (accelerometer + gyroscope)  
- **Pmod SONAR**: for obstacle detection  

> For more detailed information, see the [Wiki section](https://github.com/Artal44/Grisp_robot/wiki) of this repository.

---

## Repository Structure

```plaintext
Hardware_robot/       - 3D print and laser cut files for building the robot
Laptop_controller/    - Python server and user interface for remote control
LilyGo_software/      - Firmware for the LilyGO LoRa32 boards (communication layer)
Wiki_image/           - Images used in the wiki and documentation
balancing_robot/      - Erlang code for the stabilization and mobility system of the robot
electronics/          - Schematics and PCB files for the custom board

