#!/bin/bash
cd /sys/kernel/config/usb_gadget/
mkdir -p g1
cd g1

echo 0x046d > idVendor  # Logitech VID (or your own)
echo 0xc21d > idProduct # Example PID

echo 0x0100 > bcdDevice
echo 0x0200 > bcdUSB

mkdir -p strings/0x409
echo "123456" > strings/0x409/serialnumber
echo "OpenFFB" > strings/0x409/manufacturer
echo "PiFFB Wheel" > strings/0x409/product

mkdir -p configs/c.1/strings/0x409
echo "Config 1" > configs/c.1/strings/0x409/configuration
echo 120 > configs/c.1/MaxPower

# --- HID Function ---
mkdir -p functions/hid.usb0
echo 1 > functions/hid.usb0/protocol
echo 1 > functions/hid.usb0/subclass
echo 64 > functions/hid.usb0/report_length

# HID Descriptor (Input: 1x 16-bit axis, Output: FFB effect ID + magnitude)
echo -ne "\
\x05\x01\x09\x04\xa1\x01\
\xa1\x00\x09\x38\x15\x00\x26\xff\x03\x75\x10\x95\x01\x81\x02\xc0\
\x05\x09\x19\x01\x29\x0c\x15\x00\x25\x01\x75\x01\x95\x0c\x81\x02\x75\x04\x95\x01\x81\x03\
\x05\x0f\
\x09\x92\x15\x01\x25\x28\x75\x08\x95\x01\x91\x02\
\x09\x70\x15\x00\x26\xff\x00\x75\x08\x95\x01\x91\x02\
\x09\x73\x15\x00\x26\xff\x00\x75\x08\x95\x01\x91\x02\
\x09\x74\x15\x00\x26\xff\x00\x75\x08\x95\x01\x91\x02\
\x09\x96\x15\x00\x25\x02\x75\x08\x95\x01\x91\x02\
\x09\x97\x15\x00\x26\xff\x00\x75\x08\x95\x01\x91\x02\
\xc0" > functions/hid.usb0/report_desc

ln -s functions/hid.usb0 configs/c.1/

ls /sys/class/udc > UDC
