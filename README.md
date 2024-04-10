# Minuteman

Minuteman is an alarm clock built on the [esp-idf](https://github.com/espressif/esp-idf) platform.

## Installation

This installation assumes the esp-idf toolchain >= v5.2 is already installed and found in $IDF_PATH

```sh
git clone git@github.com:ccarral/minuteman.git --recurse-submodules
cd minuteman
idf.py menuconfig
idf.py build flash
```

## Features

- [x] Seven segment display control with MAX7219
- [x] SNTP Sync over wifi
- [x] Rotary encoder input for editing alarm times
- [ ] Snooze function
- [ ] PWM for sound generation
