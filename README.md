# Demo-Software
Software Framework for RAMMP May 2026 Demo

## Setup Instructions
1. Set up your ssh keys in github [Generate and add ssh keys for Github](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)

## Connecting to Teensy
1. Install [Arduino IDE](https://www.arduino.cc/en/software/)
2. Download the [Linux UDEV rules](https://www.pjrc.com/teensy/00-teensy.rules) for Teensy boards
3. Copy the UDEV rules file to /etc/udev/rules.d
```[bash]
sudo cp 00-teensy.rules /etc/udev/rules.d/
```
4. Download the [Teensyduino installer](https://www.pjrc.com/teensy/td_download.html)
5. Run the installer by adding execute permission and then execute it
```[bash]
chmod 755 TeensyduinoInstall.linux64
./TeensyduinoInstall.linux64
```
6. In the Arduino IDE, you can now select Teensy 4.1 as the target device and program it