# Software 

## Project descriptions

Most of the projects have UART debug statements, so you can open up a serial monitor for printf debugging/updates. 
Install the [drivers](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads) if you have 
issues seeing the COM ports after connecting the board via USB. 
 
- Smoketest: Exercises all peripherals on the board and prints any output to the serial monitor. 
- SPIWS: Main demo project that contains the minimum amount of code necessary to display a static color or single animation on the light strip. No Radio code. 
- WirelessBatteryMonitor and WirelessBatteryReceiver: Diagnostics tools that sends current battery/supply voltages to WirelessBatteryReceiver. 
It also powers the lights strips in a seperate thread to provide a current draw. 

### Legacy/Outdated/For Reference Only

These projects may not be updated for the latest revision of the boards. 

- ControllerSPI and ReceiverSPI: Legacy project that demonstrates sending light strip data over RF to a receiver running ReceiverSPI
- LightDemo: Legacy project for communicating with the light strips
- rfPacketRx*: Legacy/Outdated

## Hex image file generation settings (armhex in CCS)

The hex file is used to flash over UART/Serial, using the bootloader backdoor, which needs to be triggered manually through Simplicity Studio at the moment. 

- Memory width set to 8 (`--memwidth=8`)
- Intel format (`--intel`)
- Diagonal wrapping off (`--diag_wrap=off`)