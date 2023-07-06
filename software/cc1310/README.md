# Software 

## Project descriptions

Most of the projects have UART debug statements, so you can open up a serial monitor for printf debugging/updates. 

- ControllerSPI and ReceiverSPI: Legacy project that demonstrates sending light strip data over RF to a receiver running ReceiverSPI 
- LightDemo: Legacy project for communicating with the light strips
- SPIWS: Main demo project that contains the minimum amount of code necessary to display a static color or single animation on the light strip. No Radio code. 
- WirelessBatteryMonitor and WirelessBatteryReceiver: Diagnostics tools that sends current battery/supplhy voltages to WirelessBatteryReceiver. 
It also powers the lights strips in a seperate thread to provide a current draw. 
- rfPacketRx*: Legacy/Outdated

## Hex image file generation settings (armhex)

The hex file is used to flash over UART/Serial, using the bootloader backdoor, which needs to be triggered manually through Simplicity Studio at the moment. 

- Memory width set to 8 (`--memwidth=8`)
- Intel format (`--intel`)
- Diagonal wrapping off (`--diag_wrap=off`)