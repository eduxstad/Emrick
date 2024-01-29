
# Transmitter

---

## Summary

This project will contain all necessary code to operate all of the transmitter board.

## Main file structure

- main_tirtos - Main class of the project. Mostly used for initialization and starts the main thread.
- transmitter - Contains the main thread function. Contains the logic and control to send out rf packets.
- logger - Contains functions that interact with the flash storage. These functions are not been well tested and likely contain memory bugs. These need to be debugged if not completely rewritten.

## Other relevant files

- board.h - Contains some relevant definitions
- CC1310_LAUNCHXL_TIRTOS.cmd - Contains definitions that determine memory structure
- Under Includes: /ti/simplelink_cc13x0_sdk_4_20_02_07/source/ - contains drivers that can be useful to reference
- /ti/simplelink_cc13x0_sdk_4_20_02_07/source/ti/devices/cc13x0/startup_files/ccfg.c - contains definitions relating to setting certains pins and settings. Any changes to this file must be noted on this README as changes to drivers will not be reflected on GitHub.


## Required changes to ccfg.c

-
