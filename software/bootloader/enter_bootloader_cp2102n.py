# enter_bootloader_cp2102n.py
# ===========================
# (i) This script may need to be run as root (sudo) on Linux to access the USB device
# ===========================
# Assumptions
# -----------
# 1. GPIO 0 of the cp2102n is connected to WMCU_RESET, and the reset is active low.
# 2. BOOTLOADER_BACKDOOR (SET_CCFG_BL_CONFIG_BL_PIN_NUMBER in ccfg.c) is 0xd (GPIO-13) and is active low.
# These can be adjusted by changing the mask (first 8 bits, GPIO 7-0) and value (second 8 bits, GPIO 7-0)
# of wIndex.

import time
import usb.core
import usb.util

PID = 0xea60
VID = 0x10c4

dev = usb.core.find(idVendor=VID, idProduct=PID)
if not dev:
    print("CP210x was not found :(")
    exit(1)
print("Found CP210x")

# Section 5.29.1 of Silicon Labs AN571.pdf
reqType = 0x41 # WRITE_LATCH
bReq = 0xFF # VENDOR_SPECIFIC
wVal = 0x37E1 # CONSTANT from AN571

print("Setting reset and bootlaoder low")
wIndex = 0x0011 # turn off gpio 0 and 4
dev.ctrl_transfer(reqType, bReq, wVal, wIndex, [])

time.sleep(1) # Wait for the board to reset. This probably waits a couple orders of magnitude longer than necessary.

print("Setting reset high, rebooting into bootloader")
wIndex = 0x0101 # turn on gpio 0, leave 4 off. This boots into bootloader mode
dev.ctrl_transfer(reqType, bReq, wVal, wIndex, [])

time.sleep(1) # Wait to boot into the bootloader. This probably waits a couple orders of magnitude longer than necessary.

wIndex = 0x1010 # turn on gpio 4, we should already be in bootloader mode.
dev.ctrl_transfer(reqType, bReq, wVal, wIndex, [])

exit(0)

# print("Toggling reset to leave bootloader mode after 3 seconds")
# time.sleep(3)
# # This is where you would want to program the microcontroller with Flash Programmer 2
# # or a custom script.
# wIndex = 0x0001 # turn off gpio 0 
# dev.ctrl_transfer(reqType, bReq, wVal, wIndex, [])
# wIndex = 0x0101 # turn on gpio 0 
# dev.ctrl_transfer(reqType, bReq, wVal, wIndex, [])
# 
# print("Done")
