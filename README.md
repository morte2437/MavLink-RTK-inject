# MAVLink RTCM Injector

This Python script reads NMEA GPS data and injects RTCM correction data from a MAVLink stream into a connected GPS receiver. It is designed to run on platforms like the Raspberry Pi for use with Pixhawk flight controllers and compatible GPS modules.

## Features

- Parses and logs `$GNGGA` and `$GNRMC` NMEA sentences.
- Translates parsed GPS data into `GPS_INPUT` MAVLink messages.
- Injects RTCM correction data received from MAVLink (`GPS_RTCM_DATA`) into the GPS via serial.
- Supports high-baudrate communication with GPS modules and flight controllers.
- Optional logging of raw GPS data to file.

## Use Case

Useful for integrating GPS RTK corrections into an unmanned aerial system (UAS) or embedded setup, where MAVLink serves RTCM messages and GPS positioning data needs to be forwarded in real-time.

## Requirements

- Compatible GPS module supporting NMEA input and RTCM correction
- MAVLink-compatible flight controller (e.g., Pixhawk)
- Serial connection between Raspberry Pi (or similar device) and both GPS and FC

