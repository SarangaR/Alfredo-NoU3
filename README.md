# Alfredo_NoU3
_Library version 1.0.0

<img src="https://github.com/AlfredoElectronics/alfredoelectronics.github.io/blob/master/images/nou2-1.png" width="400px">

Library for the Alfredo NoU v3 Supports motors and servos and has helper methods for different drivetrain types.

## Getting started

### Basic setup
### Driving a robot

[AlfredoConnect](https://github.com/AlfredoElectronics/AlfredoConnect-Desktop/releases) is a driver station that runs on Windows for controlling robots over Bluetooth using your computer's keyboard or connected gamepads. It has a corresponding Arduino library, [AlfredoConnect-Receive](https://github.com/AlfredoElectronics/AlfredoConnect-Receive), which contains examples for using AlfredoConnect with the Alfredo NoU2. After following the [basic setup tutorial](#basic-setup) here, follow the [getting started tutorials for AlfredoConnect-Receive](https://github.com/AlfredoElectronics/AlfredoConnect-Receive#getting-started) to learn how to control your robot using AlfredoConnect.

## Troubleshooting

### Can't connect to the ESP32 over USB to upload
* The most common cause of this is accidentally using a power-only micro USB cable, which won't allow the computer and ESP32 to communicate, but will power on the ESP32. Try a new cable. When plugging in the ESP32, you should hear a sound on the computer indicating it's plugged in.
* Make sure you're selecting the right COM port. The most common way to check is to unplug the ESP32, check the ports list in the Arduino IDE, (`Tools` > `Port`), plug the ESP32 in, and then check the list again to see what COM port got added. On Windows, you can also open Device Manager (`Win+R`, type in `devmgmt.msc`) and check the `Ports (COM & LPT)` dropdown, which should include a port list similar to the one in the Arduino IDE.
* You may need to install appropriate drivers for [CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) or [FTDI](https://ftdichip.com/drivers/vcp-drivers/), depending on your ESP32 module type. If unsure, get both.

## Links

* [**MiniFRC Discord**](https://discord.gg/VtGvf6B): This is the best place to get quick help with the Alfredo NoU2 and this library from both users and the Alfredo Electronics team.
