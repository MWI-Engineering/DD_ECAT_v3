# DD_ECAT_v3

## Schematic layout 
- Based on: https://github.com/Ultrawipf/OpenFFBoard/wiki
![DD_ECAT_layout_v1](https://github.com/user-attachments/assets/a7c4d775-17ac-4dd2-baf8-e214e4249a90)

## Concept idea

- I want to use a Raspberry Pi 4B, that can connect as a USB HID Joystick with FFB functionality. With the FBB commands from the game via something like: Windows.Gaming.Input.ForceFeedback.
- The script: create_ffb_gadget.sh makes the Pi to show up as a USB HID. But im not sure if the FFB inputs are correct.
- I created a basic hid_interface.c and .h that should get the right commmands from the games i am playing. 
- This should send the commands to the ffb_calculator. These recalculated commands should be send to the SOEM interface.
- This SOEM interface start up the connection using EtherCAT with the servomotor and intergrated encoder en sends torque, position and velocity commands to the motor and reads the actual shaft position and sends this back via the SOEM interface to the FFB axis/effects calculator.
- All of this is regulated via the main.c script. 

## To make it work
- All the scripts must be compiled on the raspberry pi.
- 
