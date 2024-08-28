# ESP32 BME280 driver/library

This is a single-file header-only C++ style library (see files in `main/include/`) for driving the **BME280** Bosch's humidity sensor from an **ESP32** microcontroller.

Currently supported are:
- BME280


It uses the official Bosch API:
- https://github.com/boschsensortec/BME280_SensorAPI


## Installation & usage
See example in `main/main.cpp`.

- Move the files from `main/include/` to your include directory.
- Install `boschsensortec/BME280_SensorAPI` as component.
- #include the `BME280.h` in your code.
- Create an object.
- Done!
