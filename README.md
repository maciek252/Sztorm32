An attempt to deliver an open-source firmware for the gimbal Feiyutech WG2X
===

## The general idea

The gimbal implements the concept on which the free (but closed source) gimbal firmware T-STorM32 is based: the motor boards for yaw, roll and pitch axes are linked by a daisy-chain link over the high-speed serial port. Contrary to T-STorM32, here there is is separate "motherboard" (it's integrated with the yaw board) and also 


### the development plan

- investigating the circuitry (mostly done)
- checking if the MCUs can be replaced (confirmed)
- setting up the customized Chibios program (done)
- enabling the serial ports (done)
- enabling SPI for the magnetic encoder (done)
- enabling the timers to run the motor driver chip (done)
- adding a simple non-FOC block motor driver software
- enabling the ADC converters for the current sensors
- adding a FOC-based motor driver software, using the encoder and current sensors
- enabling the IMU on the motherboard
- enabling the roll and pitch boards
- enabling the SPI-based IMU connected to the pitch board
- implementing the NT bus protocol for the serial links between the modules
- implementing the gimbal algorithm

- checking how to flash the firmware onto ESP32, if this component is write protected and needs to be replaced
- restructuring the code to allow for different MCU types and all the modules





### Sztorm32-Lite

This is a simplified version of the project, not changing the internal WG2X implementation of the gimbal part, but taking over the control.

The project aims to provide a dedicated firmware for ESP32 Vroom. At the current stage, it is possible to receive the gimbal status and issue commands by an USB-TTL interface, soldered as explained in the following photo:

![Alt text](photo/WG2Xboard.jpg?raw=true "Title")

after connecting the interface and selecting the baudrate of 115200, the python script Sztorm32-Lite/pyserial1.py can be used for issuing commands, and the telemetry from the gimbal will be received.

