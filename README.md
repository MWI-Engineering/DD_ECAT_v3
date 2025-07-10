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

### Configure the pi

#### Phase 1: Raspberry Pi Setup

- Flash OS: Flash Raspberry Pi OS Lite (64-bit) to your SD card using the Raspberry Pi Imager.
- Initial Boot & Config:
Boot the Pi. Connect it to your network via its Ethernet port for now. SSH into it.
Run sudo raspi-config to set your locale, timezone, and enable SSH.

- Update your system:

sudo apt update
sudo apt full-upgrade -y

- Install Essential Tools:

sudo apt install -y build-essential git

- Isolate a CPU Core (Optional but Recommended): For better real-time performance, you can dedicate a CPU core to the main control loop. Edit the boot command line:

sudo nano /boot/cmdline.txt

Add isolcpus=3 to the end of the line. This reserves the 4th core (core #3) for our application. Reboot after saving.

#### Phase 2: EtherCAT Master Setup (SOEM)

- Clone SOEM:
cd ~
git clone https://github.com/OpenEtherCATsociety/SOEM.git

##### Build SOEM

- cd SOEM
- mkdir build
- cd build
- cmake ..
- make
- sudo make install

##### Configure Network Interface

The EtherCAT master needs raw socket access to the Ethernet port. We'll use eth0.
Find your Pi's MAC address: ip a (look for link/ether under eth0).
Connect the Synapticon drive directly to the Pi's Ethernet port. Power on the Synapticon drive (both logic and motor power).

##### Test Communication

SOEM comes with example tools. Let's use slaveinfo to see if the Pi can find the drive.
Navigate to the build directory where the test executables are: ~/SOEM/build/test/linux/slaveinfo/
Run the test. You MUST use sudo because it requires raw network access.

sudo ./slaveinfo eth0

If successful, you will see output like:
SOEM (Simple Open EtherCAT Master)
Slaveinfo
Starting slaveinfo
ec_init on eth0 succeeded.
1 slaves found and configured.
Slave 1
 MAN: 00000abc ID: 12345678 REV: 00000001
 State: PREOP
 ... (more info)
This step is critical. If it fails, do not proceed. Check your cabling, power, and network interface name.

#### Phase 3: USB HID Gadget Setup

This phase makes the Pi appear as a joystick to your PC.

##### Enable libcomposite:

- echo "dtoverlay=dwc2" | sudo tee -a /boot/config.txt
- echo "libcomposite" | sudo tee -a /etc/modules

##### Create the HID Gadget Script

We need a script that defines the joystick's capabilities (1 axis for steering, FFB support). Create a file named create_ffb_gadget.sh.

- **Note to self:** Paste instructions how I made this .sh file and automatically starts up on boot.

##### Automaticlly start the script

- use this command: sudo nano /etc/systemd/system/ffb-gadget.service
- Paste this:
{
[Unit]
Description=Create USB Gadget for FFB Wheel
After=network.target sys-kernel-config.mount
Requires=sys-kernel-config.mount

[Service]
ExecStart=/usr/bin/create_ffb_gadget.sh
Type=oneshot
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
}

- Than enable and start it:
{sudo chmod +x /usr/bin/create_ffb_gadget.sh
sudo systemctl daemon-reexec
sudo systemctl enable ffb-gadget.service
sudo reboot
}

#### Phase 4: Clone git project

- Clone DD_ECAT_v3:
cd ~
git clone https://github.com/Mwi93/DD_ECAT_v3.git

##### to do:

- Update the create_ffb_gadget.sh so it will work = done
- Update hid_interface.c with aditional FFB cases = done
- Update ffb_calculator.c with the expanded ffb_calculator.h.= done
- Update soem_interface.c and .h with the new ffb_calculator and hid_interfaces = done
- Update main.c = done
- **Next steps:**
- Check via claude.ai (pro) all files if it works all together correctly.
- See how much work it is to make a GUI where the parameters can be adjusted. (Perhaps not possible due the files need to be compiled)
- Try to create a good make file on the following scripts in phase 5.
- For now use the steps in Phase 5, will update the make file when code is working.

#### Phase 5: Install all files

- All the scripts must be compiled on the raspberry pi.
- sudo gcc -c soem_interface.c -o soem_interface.o -I/usr/local/include/soem
- sudo gcc -c hid_interface.c -o hid_interface.o
- sudo gcc -c ffb_calculator.c -o ffb_calculator.o
- sudo gcc -c main.c -o main.o
- Link ALL the object files and necessary libraries using this:
- sudo gcc main.o \
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
