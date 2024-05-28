# Energy based controller for real inverted pendulum stand

## Overview
The "Energy based controller for real inverted pendulum stand" project is aimed to test traditional cart-pole system control method such as PD control and Energy based control in real life.

- Course: Advanced Control Methods, Skoltech, 2024
- Team Members: Sergey Bakulin, Ruslan Babakyan, Artem Bazhenov
- Final Presentation: https://docs.google.com/presentation/d/1UkfdIFDTp4hr2HpFwEp8onsQ0ok_Wjc1/edit?usp=sharing&ouid=100422232997772233481&rtpof=true&sd=true

---

## Table of Contents

- [Overview](#overview)
- [Problem Statement](#problem-statement)
- [Results](#results)
- [Run the Project](#run-the-project)
- [Other Section](#other-section)
- [Bibliography](#bibliography)

---

## Problem Statement
The project includes such key objectives as:
- Developing the low-level software for the for the real inverted pendulum stand (RIPS)
- Holding the pendulum in an vertical position for RIPS
- Making the pendulum swing
- Solvng the problems of limiting the linear motion of the pendulum

---

## Results
The outcomes of the project are:
- The Inverse Pendulum Stand was built
- The Low level software for the Stand was developed
- The High level communication was created
- PD controller for the pendulum angular position was developed and Energy-based controller were implemented


---


## Run the Project
- Clone this repository
- Open Cube IDE projecte from "STM32 software" folder
- Upload firmware to the STM32 from Cube IDE
- Connect USB-UART converter to the PC

### Requirements
 - Python 3
 - Numpy
 - PySerial

### Setup and Installation
Instructions for setting up the project environment, which may include:
- Installing dependencies: `pip install -r requirements.txt`
- Setting up a virtual environment
- Connect USB-UART converter to the PC
- Define the number of the COM port in the system

### Running the Code
Exact commands to execute the project, such as:
```bash
python main.py <COM PORT>
```

---

## Bibliography

- Tran, Ashley & Pepitone, Paola & Choi, Jun. (2019). State Space and Energy Based Control for an Autonomous Self-Rising Inverted Pendulum. 10.13140/RG.2.2.36425.70243. 