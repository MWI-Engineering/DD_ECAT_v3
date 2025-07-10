# DD_ECAT_v3

## Schematic layout

- Based on: https://github.com/Ultrawipf/OpenFFBoard/wiki
- ![DD_ECAT_layout_v1](https://github.com/user-attachments/assets/eae2e943-441e-4428-8cb8-4dae76ef00f0)

## Concept idea

- I want to use a Raspberry Pi 4B, that can connect as a USB HID Joystick with FFB functionality. With the FBB commands from the game via something like: Windows.Gaming.Input.ForceFeedback.
- Using SOEM on my raspberry pi 4B to make it a EtherCAT master that sends commands to the Synapticon ACTLINK-S servo motor and sends the actual position (encoder value) back to SOEM.
- I have the PDO mapping available in the PDO_mapping.txt file.

## What i have now

- The script: create_ffb_gadget.sh makes the Pi to show up as a USB HID. But im not sure if the FFB inputs are correct.
- I created a basic hid_interface.c and .h that should get the right commmands from the games i am playing. 
- This should send the commands to the ffb_calculator. These recalculated commands should be send to the SOEM interface.
- This SOEM interface start up the connection using EtherCAT with the servomotor and intergrated encoder en sends torque, position and velocity commands to the motor and reads the actual shaft position and sends this back via the SOEM interface to the FFB axis/effects calculator.
- All of this is regulated via the main.c script.

## To make it work

- All the scripts must be compiled on the raspberry pi.
- gcc -c soem_interface.c -o soem_interface.o -I/usr/local/include/soem
- gcc -c hid_interface.c -o hid_interface.o
- gcc -c ffb_calculator.c -o ffb_calculator.o
- gcc -c main.c -o main.o
- Link ALL the object files and necessary libraries using this:
- gcc main.o \
    soem_interface.o \
    hid_interface.o \
    ffb_calculator.o \
    -o my_ethercat_app \
    -L/usr/local/lib \
    -lsoem \
    -lpthread \
    -lrt \
    -lusb-1.0 \
    -lm
- These steps must be tested, i've not come this far yet.
- 
