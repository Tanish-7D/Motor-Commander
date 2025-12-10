# Motor-Commander

Stepper Motor Commander using the messages for the computer and controlling the stepper motor through PWM signals

---

## Purpose
The main focus is:
1. **Hardware** — list of required components for the system and establish communication between the computer and the Raspberry Pi 4
2. **Installation** — get the software running on the ROS 2 Jazzy system on the Raspberry Pi 4.
4. **Usage** — run node to send PWM signals to the motor based on a proportional calculation
5. **Debugging** — troubleshoot and verify correct operation.

---

The system relies on the following hardware:
- **Motor** — We use a NEMA 17 stepper motor
- **Microcontrollers** — RaspberryPi4 drives the stepper motors (NEMA 17)
- **Motor Controller** — MicroStepper Driver for the WASP Extruder <br> <br>
<img width="350" height="300" alt="image" src= "https://github.com/user-attachments/assets/6547f814-3ed1-442b-a861-d817cb965ae6" /> <br>

NOTE: We use these Digital Pins on the RPi: PULSE is 17, DIR is 27, and EN is 22

---

