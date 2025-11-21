
## Deployment steps


- for a new collar the lora device id needs to be registered on chirpstack and the firmware needs to be installed on the device

step 1 - get the device id 

- use the tools/lora_dev sketch

1. `cd active-tracker/tools/lora_dev`

2. if the uf2 file has not been built yet,
- `mkdir build && cd build`
- `cmake -DPICO_BOARD=pimoroni_tiny2040 ..`
- `make`

Otherwise just go to the build folder
- `cd build`

3. connect the board with usb-c (has to be data capable cable) while holding the BOOT button. You should see a new mass storage device appear on your computer called RPI-RP2.

4. copy the `lora_dev.uf2` file to the RPI-RP2 device.
`cp lora_dev.uf2 /Volumes/RPI-RP2/` (on macOS, adjust for your OS accordingly)
The board will reboot and run the lora_dev program.

5. use a serial monitor program to read the output of the board. e.g. using `tio`:

`tio --auto-connect new`

Note the DevEui


step 2 - register the device on chirpstack

1. Login to chirpstack web interface

2. Go to "Applications" and select WILDEBEEST

3. Add device

4. Name W202x_xxx (where xxx is the collar number and 202x is the year of registration)

5. Enter the DevEui from step 1

6. Select device profile ACTIVITY_TRACKER_V2_2

7. Add the application key:

434F4C494E5357494C44454245455354

step 3 - install the firmware on the device

CONNECT THE ANTENNA TO THE DEVICE BEFORE POWERING OR IT MAY DAMAGE THE LORA MODULE
1. `cd active-tracker/firmware`

2. if the uf2 file has not been built yet,
- `mkdir build && cd build`
- `cmake -DPICO_BOARD=pimoroni_tiny2040 ..`
- `make`
Otherwise just go to the build folder
- `cd build`

3. reset the board while holding the BOOT button. You should see a new mass storage device appear on your computer called RPI-RP2.

4. copy the `pico_tracker.uf2` file to the RPI-RP2 device.

5. reboot the device and check chirpstack for join

