# GRiSP2 Robot

## Introduction

This project extends the master thesis by François Goens and Cédric Ponsard:  
[*Dynamic balancing in the real world with GRiSP*](http://hdl.handle.net/2078.1/thesis:48907).  
It is developed by **Arthur Dandoy** and **Sam Raymakers**.

While the original project focused on building a self-balancing robot, this thesis reuses the same architecture to design a **smart, stable, and mobile table** that:

- maintains **static and dynamic balance**,
- navigates **autonomously** inside a room,
- integrates **obstacle avoidance**,
- is driven by a **GRiSP2 board** and **Hera framework** using sensor fusion (Kalman + PID).

The system is implemented in **Erlang/OTP 27.2.4**, and leverages fault-tolerant patterns using OTP supervisors. For details on the Hera platform, see [hera](https://github.com/stritzinger/hera).

---

## Repository Structure

```plaintext
Hardware_robot/         - 3D print + laser cut files for the mobile table hardware
Laptop_controller/      - Python UI to remotely control the robot (mandatory to operate)
LilyGo_software/        - Code for the LoRa32 LilyGO modules (robot ↔ controller communication)
Wiki_image/             - Media used for documentation
balancing_robot/        - Core Erlang logic: balance control, PID, Kalman, robot FSM
electronics/            - Custom PCB design files

