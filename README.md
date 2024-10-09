# Alfredo_NoU3

<img src="https://github.com/AlfredoSystems/alfredosystems.github.io/blob/master/images/nou3-1.png" width="400px">

Library for the Alfredo NoU3. Supports motors and servos, has helper methods for different drivetrain types.

## Getting started
1. **Get the Arduino IDE.** This lets us write and upload programs to our robot. Download it from the [Arduino website](https://www.arduino.cc/en/main/software) and install it.
2. **Open the Library Manager** In the Arduino IDE, open the Library Manager by clicking the books icon on the left.
3. **Install the** `Alfredo-NoU3`_ **library.** Click *Filter your search...* and type **Alfredo-NoU3**, then click **INSTALL**.
4. **Configure the Arduino IDE to upload to a NoU3.** In the Arduino IDE, click **File** > **Preferences**. Paste the following in the **Additional Boards Manager URLs** field.
`https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json``
Then, go to **Tools** > **Board** > **Boards Manager**. Search for "ESP32" and install the package "esp32 by Espressif Systems".

## Driving a robot
* check out the NoU3 read-the-docs for the full robot programming tutorial

## Troubleshooting
* The most common cause of this is accidentally using a power-only micro USB cable, which won't allow the computer and ESP32 to communicate, but will power on the ESP32. Try a new cable. When plugging in the ESP32, you should hear a sound on the computer indicating it's plugged in.
