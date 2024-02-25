# xid_wheel_adapter
A DIY adapter that can receive input from a logitech wheel and output as an (original/classic) Xbox (XID) wheel.

As a bonus it will also make some xbox/xinput controllers work on the original xbox. Console will still see it as a wheel but most games will work.

### PRs are welcome!
There's some stuff that still needs to be added and some QoL improvements. Would also like to map rumble to some kind of FFB effect.
Let me know if you can help.

## Supported devices
* WingMan Formula Force GP (GT Force)
* Driving Force
* Driving Force Pro (GT Force Pro)
* Driving Force GT
* G25 Racing Wheel
* G27 Racing Wheel
* G29 Racing Wheel

## Mapping
| Wheel      | Xbox              |
|------------|-------------------|
| Dpad       | Dpad              |
| Wheel axis | Left stick X      |
| Gas axis   | Right trigger     |
| Brake axis | Left trigger      |
| Select     | Back              |
| Start      | Start             |
| Cross, L1  | A                 |
| Circle     | B                 |
| Square, R1 | X                 |
| Triangle   | Y                 |
| L2         | Black             |
| R2         | White             |
| L3         | Left Stick click  |
| R3         | Right Stick click |

It will map the paddle shifters as:<br/>
Left paddle = A button (gear down)<br/>
Right paddle = X button (gear up)<br/>
 
Most xbox games uses this configuration. Seems to be the mode used by the Mad Catz MC2 Wheel.

## Building
Requires a Raspberry Pi Pico (RP2040) board and a USB Type-A port for input.

Check the wiring guidance [here](https://github.com/sekigon-gonnoc/Pico-PIO-USB/discussions/7).

USB Pins can be changed. Just need to set them in code.

Define the `D+` pin on sketch. `D-` will be `D+` + 1.

Required configuration are on the main sketch file as `PIN_USB_HOST_DP`

Firmware builds under Arduino IDE.

Required libs. Install using Arduino IDE.

[arduino-pico (3.1.1)](https://github.com/earlephilhower/arduino-pico#installing-via-arduino-boards-manager)<br/>
[Pico-PIO-USB (0.5.3)](https://github.com/sekigon-gonnoc/Pico-PIO-USB)<br/>
[Adafruit_TinyUSB_Arduino (2.2.6)](https://github.com/adafruit/Adafruit_TinyUSB_Arduino)

Configure IDE as:
* Board: Raspberry Pi Pico
* CPU Speed: 120MHz
* USB Stack: Adafruit TinyUSB

## Ready to use binaries
~~Don't want to build from source? Check the releases page.~~
Will be released shortly.

## Credits
Logitech USB input code from my other project.
[lgff_wheel_adapter](https://github.com/sonik-br/lgff_wheel_adapter)

Xbox (XINPUT/XID) input/output [ogx360_t4](https://github.com/Ryzee119/ogx360_t4) from Ryzee119.

The [XID docs](https://xboxdevwiki.net/Xbox_Input_Devices) from xboxdevwiki.

## Disclaimer

Code and wiring directions are provided to you 'as is' and without any warranties. Use at your own risk.