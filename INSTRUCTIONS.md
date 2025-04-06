## Flashing the board

1. Connect the board to your PC through the USB-C labeled esp32c6. 
2. Run `cargo run --release`
3. Disconnect the board from your PC once the board is flashed

## Setup the environment 

1. Add the serial-ETH adapter device to your udev rules:

```
# echo 'ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"' > /etc/udev/rules.d/10-esp32c6.rules 
```
2. Reload udev rules:

```
# udevadm control --reload-rules
```

3. Connect your PC to to the serial-ETH adapter using the USB-C port,
the device will most likely appear as `/dev/ttyUSB0`

4. Open up minicom, with baud rate set to 115200 and the appropriate device file.
You should also setup logging to file, e.g. to a file named `my.log`
```
minicom -C my.log -b 115200 -D /dev/ttyUSB0
```
5. In minicom, press `ctrl-z` and then `u`.
This is necessary to prevent mangled output, namely
it will automatically add carriage return `\r` to each newline `n`
the board sends

## Run the firmware

1. Connect the board to power by:
  - Connecting the ETH cable to the board and the adapter, then
  - Connecting the battery to the adapter

2. You should begin to see the output from the board.
Wait a few seconds until you are asked to input the reference weight necessary
to calibrate the load cells. Do not input the weight yet.

3. Place an object and input its weight in kilos either through serial or bluetooth
The format is a floating point number, so enter `1.2` if the object weight 1.2kgs

4. After a few seconds output from load cell will begin to appear. Wait a few
more until the output from the pressure sensor begins to appear.

5. Once you are ready to detonate the fuze, write `fuze` over serial or bluetooth.
