
# Receiver

---

## Summary

This project will contain all necessary code to operate all of the receiving light strips. For readability, the code is split into multiple files by feature implemented.

## Main file structure

- main_tirtos - Main class of the project. Mostly used for initialization and starts the main thread.
- receiver - Contains the main thread function which controls the execution flow of the program.
- packetReceive - Contains the logic and control to operate the RF radio. Functions in the file run on a child thread of the main thread.
- patterns - Contains the functions that control the LED patterns as well as the logic to switch between the. Also contains old light patterns that may or may not be used in the future.
- WS2812 - Contains driver functions for controlling the LED strips.
- RFQueue - Contains queue data structure for storing received RF packets.
- logger - Contains functions that interact with the flash storage. These functions are not been well tested and likely contain memory bugs. These need to be debugged if not completely rewritten.

## Other relevant files

- board.h - Contains some relevant definitions
- CC1310_LAUNCHXL_TIRTOS.cmd - Contains definitions that determine memory structure
- Under Includes: /ti/simplelink_cc13x0_sdk_4_20_02_07/source/ - contains drivers that can be useful to reference
- /ti/simplelink_cc13x0_sdk_4_20_02_07/source/ti/devices/cc13x0/startup_files/ccfg.c - contains definitions relating to setting certains pins and settings. Any changes to this file must be noted on this README as changes to drivers will not be reflected on GitHub.


## Required changes to ccfg.c

-
