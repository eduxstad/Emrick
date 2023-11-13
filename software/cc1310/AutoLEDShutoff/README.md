
# Smoketest

---

## Summary

This smoketest is intented to test and demonstrate that all features of the
Emrick board are working, ideally without creating any smoke. The program
attempts to print any errors to the UART/serial console. 

## Peripherals Exercised

* Serial over UART - Console output is printed to serial at baud 115200
	(You may have to install the Silicon Labs [drivers](
	https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads) 
	to open the COM port).
* Supply Voltage - Internal supply voltage monitor of the CC1310.
* Battery Voltage - ADC measurement of the battery voltage (floats if no
	battery is present).
* External Flash - Writes and reads a file from the external flash chip. 
* 5V Supply - Turns on the boosted 5V supply
* LED Strip - Displays maximum brightness (white), RGB, and an animation
	on the LED strip. Turns off after. 
* RF TX - Transmits a packet at boot. 
* RF RX - Creates a thread to listen for packets and prints whenever
	it receives a packet. You can start another board running the 
	smoketest to trigger this message. 
* Power LED - The LED should flash after initialization is complete.


## Application Design Details

TI-RTOS:

* When building in Code Composer Studio, the kernel configuration project will
be imported along with the example. The kernel configuration project is
referenced by the example, so it will be built first. The "release" kernel
configuration is the default project used. It has many debug features disabled.
These feature include assert checking, logging and runtime stack checks. For a
detailed difference between the "release" and "debug" kernel configurations and
how to switch between them, please refer to the SimpleLink MCU SDK User's
Guide. The "release" and "debug" kernel configuration projects can be found
under &lt;SDK_INSTALL_DIR&gt;/kernel/tirtos/builds/&lt;BOARD&gt;/(release|debug)/(ccs|gcc).

FreeRTOS:

* Please view the `FreeRTOSConfig.h` header file for example configuration
information.
