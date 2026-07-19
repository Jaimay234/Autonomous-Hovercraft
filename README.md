# Autonomous-Hovercraft
Hovercraft project that was designed to autonomously navigate a maze. The system directly interfaces with onboard sensors and other hardware components over UART and I/O driven inteerrupts. The main goal of the project was to complete a course without human input.

Team of 5, placing 3rd out of about 30 teams.


## Overview
The hovercraft uses low level embedded programming with real time sensor input to make autonomous navigation decisions. Scored not only upon completion but also on efficiency and simplicity so as to add realistic constraints. Every added component or external library came at the cost of points favouring simplitistic and lean code.

## Theory
| Component | Description |
|---|---|
| Communication | UART protocol for sensor/peripheral data exchange |
| Control Flow | Interrupt service routines drive real-time behavior |
| Decision Logic | C++ logic interprets sensor input and determines hovercraft behavior |
| Hardware | Direct register-level programming, no vendor abstraction layers |

## Notes
This project was built under tight hardware and grading constraints, so some design choices favor simplicity over robustness. More detailed documentation on the hardware setup, wiring, and specific sensor behaviors can be added on request.
### Video Demonstration:
[Hovercraft Run - Performance Footage](https://www.dropbox.com/scl/fi/f5khz8r4wizta8bmoyiln/Hovercraft_Run.MOV?rlkey=gj0t8ny3j5b2m7db8e2a7b1iu&st=llz0j85b&dl=0)
