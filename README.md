# Oceanographic Winch Control System

This project focuses on the development of an embedded control system for oceanographic winches designed to operate on an Autonomous Surface Vehicle (ASV).

The winches are used to deploy and retrieve underwater instruments, including a CTD sensor and an acoustic sonar, for oceanographic data collection.

The main objective was to develop the software and electronic control architecture required to operate the winches in both manual and autonomous modes, while integrating sensor feedback and safety mechanisms.

## Main Features

### 1. Manual Winch Control

Allows operators to control the winch directly from the Raspberry Pi terminal.

Features include:
* Motor control using keyboard inputs (cable deployment and retrieval)
* Immediate motor stop/pause
* Adjustable motor speed

### 2. Autonomous Winch Control

Designed to automate underwater instrument deployment and retrieval.

Main objectives:
* Deploy the instrument to a specified target depth
* Maintain the instrument at the desired depth
* Automatically retrieve the instrument
* Implement safety mechanisms during retrieval
* Enable remote control and real-time telemetry through a TCP server using the NDJSON protocol

## Project Status & Limitations

⚠️ This repository does not contain a finalized control system.

The code was developed over several months during my final-year engineering internship and provides a functional control system. However, the current implementation relies on a single sensor to estimate the depth of the deployed underwater instruments, whereas the original objective was to use two complementary sensors for more reliable and redundant depth estimation.
