
# AutoLED shutoff

---

## Summary

This project is based off of the smoketest is intented to show a simple state machine that 
turns off the LED strip when the Battery voltage is low. THE LED is configured to show
whether the battery is charging, and when the battery stops charging the power is unplugged,
the LED strip turns on. The last 50 seconds of battery voltage history is also printed to the
serial console (sampled every 5 seconds).
An implementation like this should prevent any weird shutoff behaviours or reboots when the 
battery power is low. The simple state machine should be extended for the full project.

The GPIO (PGOOD, BAT_CHG) is configured for Rev 4.

## Peripherals Exercised

* Serial over UART - Console output is printed to serial at baud 115200
	(You may have to install the Silicon Labs [drivers](
	https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads) 
	to open the COM port).
* Supply Voltage - Internal supply voltage monitor of the CC1310.
* Battery Voltage - ADC measurement of the battery voltage (floats if no
	battery is present).
* 5V Supply - Turns on the boosted 5V supply
* LED Strip - Displays maximum brightness (white).
* Power LED - The LED should flash after initialization is complete.
* PGOOD and BAT_CHG signals - shows how to configure GPIO to read these signals from the battery charger.

## State Machine

0: Shutdown: Wait for battery charge (triggered upon low bat voltage)

1: Running:  Display LEDs in white (triggered upon high bat voltage and PGOOD low)

For a real design we need more states like charging, plugged in, etc
This assumes we will always charge and discharge the battery, in real life this won't be the case.
This test designed to be connected to a power supply with a set duty cycle that fully charges the
battery and waits long enough for the battery to discharge between cycles.



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
