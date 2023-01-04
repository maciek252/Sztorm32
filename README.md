An attempt to deliver an open-source firmware for the gimbal Feiyutech WG2X
===

## The general idea

The gimbal implements the concept on which the free (but closed source) gimbal firmware T-STorM32 is based: the motor boards for yaw, roll and pitch axes are linked by a daisy-chain link over the high-speed serial port. Contrary to T-STorM32, here there is is separate "motherboard" (it's integrated with the yaw board) and also 

## The hardware

In the original hardware the MCUs are GD32 (a clone of STM32F103). Quart resonators on every board are running at 12 MHz.

### The motherboard (yaw axis)

The motherboard (and the control of the yaw axis) contains also some stuff for charging the lipo battery. It also has an ESP32 chip for controlling the wifi and bluetooth communication.

The connections with the MCU are (pin labels for STM32F103):

- Serial2 to the ESP32 (PA2, PA3)
- Serial3 to the roll module (PB10, PB11)
- Serial1: PB6, PB7 connected to the USB socket. Note that it does not connected to the DM pins (PA11, PA12)
- MQ730 magnetic encoder: SPI1 (PA6: MISO, PA7: MOSI, PA5: SCK), PA1 as the chip select (CS)
- AM2827 motor controller: 3-PWM for controlling the phases, 3PWMs for disabling the stand-by (high impedance) for every phase, PA8, PA9, PA10 for PWMs, PB13, PB14, PB15 for complementary PWMs enabling the driver
- current sensors: two resistor sensors + an operational amplifier, the MCU pins PA0 and PA4

### the pitch board


- MQ730 magnetic encoder: SPI1 (PA6: MISO, PA7: MOSI, PA5: SCK), PA1 as the chip select (CS)
- AM2827 motor controller: 3-PWM for controlling the phases, 3PWMs for disabling the stand-by (high impedance) for every phase, PA8, PA9, PA10 for PWMs, PB13, PB14, PB15 for complementary PWMs enabling the driver
- current sensors: two resistor sensors + an operational amplifier, the MCU pins PA0 and PA4
- IMU: MPU6500
    SDO -> PB4
    SDA/SDI -> PB5
    SCL/SCK -> PB3
    CS -> PB8
- Serial - 

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



### inter-module connections pinout

Yaw<->Roll:    TX RX GND GND +3.3V +3.3V +VBAT +VBAT
Roll<->Pitch   TX RX GND GND +3.3V +VBAT
Pitch<->IMU    to be checked, it's SPI + 3.3V + GND

### LEDs

There are two LEDs. Don't know yet if they are connected to the ESP32 or to some minor chips controlling the power switching. None of them seems to be connected to the STM32 MCU.

### Known issues



### Sztorm32-Lite

This is a simplified version of the project, not changing the internal WG2X implementation of the gimbal part, but taking over the control.

The project aims to provide a dedicated firmware for ESP32 Vroom. At the current stage, it is possible to receive the gimbal status and issue commands by an USB-TTL interface, soldered as explained in the following photo:

![Alt text](photo/WG2Xboard.jpg?raw=true "Title")

after connecting the interface and selecting the baudrate of 115200, the python script Sztorm32-Lite/pyserial1.py can be used for issuing commands, and the telemetry from the gimbal will be received.

