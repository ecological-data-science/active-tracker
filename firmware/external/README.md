
## External Libraries

This directory contains external libraries that are used in the firmware. These libraries are not part of the firmware codebase, but they are necessary for the firmware to function properly:
- https://github.com/raspberrypi/pico-sdk
- https://github.com/raspberrypi/pico-extras
- https://github.com/raspberrypi/pico-tflmicro


## Load submodules with 
```
git submodule update --init --recursive
```


also need to initialize the submodules in the pico-sdk and pico-extras directories
```
cd pico-sdk
git submodule update --init --recursive
cd ../pico-extras
git submodule update --init --recursive
```

