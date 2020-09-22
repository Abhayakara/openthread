# OpenThread CLI SRP Example

This example application modifies the CLI app to provide a demo of an SRP client.  These instructions walk through running on nrf52840 hardware.

## 1. Toolchain

Download and install the [GNU toolchain for ARM Cortex-M][gnu-toolchain].

[gnu-toolchain]: https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm

To install the GNU toolchain and its dependencies, run the following commands in Bash:

```bash
$ cd <path-to-openthread>
$ ./script/bootstrap
```

## 2. Build

```bash
$ cd <path-to-openthread>
$ ./bootstrap
$ make -f examples/Makefile-nrf52840 FULL_LOGS=1
```

After a successful build, the `elf` files can be found in `<path-to-openthread>/output/nrf52840/bin`. You can convert them to hex using `arm-none-eabi-objcopy`:

```bash
$ arm-none-eabi-objcopy -O ihex ot-cli-ftd ot-cli-ftd.hex
```

## 3. Flash the binary

Flash the compiled binaries onto nRF52840 using `nrfjprog` which is part of the [nRF Command Line Tools][nrf-command-line-tools].

```bash
$ nrfjprog -f nrf52 --chiperase --program output/nrf52840/bin/ot-cli-ftd.hex --reset
```

## 4. SEGGER J-Link logging


1. Connect the nrf board to your machine with a USB cable.
2. Run `JLinkExe` to connect to the target. For example:

```
JLinkExe -device NRF52840_XXAA -if SWD -speed 4000 -autoconnect 1 -SelectEmuBySN <SEGGER_ID> -RTTTelnetPort 19021
```

3. Run `JLinkRTTTelnet` to obtain the RTT logs from the connected device in a separate console. For example:

```
JLinkRTTClient -RTTTelnetPort 19021
```

## 5. CLI
1.  Find the tty name of your board:

```
ls /dev/tty*
```
Look for a usbmodem option.  For example:
```
/dev/tty.usbmodem0006839712041
```
Connect to this device using the screen command:

```shell
screen /dev/tty.usbmodem0006839712041 115200
```

Now you are connected with the CLI.

## Connect to thread network

get the dataset information from your border router:

```
wpanctl getprop thread:activedataset
thread:activedataset = [
"Dataset:Channel                  =  14" 
"Dataset:PanId                    =  0xABCD" 
"Dataset:ExtendedPanId            =  0x1234567812345678" 
"Dataset:MasterKey                =  [DEADBEEFDEADBEEFDEADBEEFDEADBEEF]" 
```
Enter the channel, panid, extended panid, and master key via CLI

```
> dataset panid 0xABCD
Done
> dataset channel 14
Done
> dataset extpanid 1234567812345678
Done
> dataset masterkey DEADBEEFDEADBEEFDEADBEEFDEADBEEF
Done
> dataset commit active
Done
> ifconfig up
Done
> thread start
Done
```

At this point the example app should connect to the thread network, find the srp-mdns-proxy information in the thread network data, and advertise its record.