# Libre Solar Charge Controller Firmware

![build badge](https://travis-ci.com/LibreSolar/charge-controller-firmware.svg?branch=master)

This repository contains the firmware for the different Libre Solar Charge Controllers based on [Zephyr RTOS](https://www.zephyrproject.org/).

Coding style is described [here](https://github.com/LibreSolar/coding-style).

**Warning:** This firmware is under active development. Even though we try our best not to break any features that worked before, not every commit is fully tested on every board before including it to the master branch. For stable and tested versions consider using the latest [release](https://github.com/LibreSolar/charge-controller-firmware/releases).

## Supported devices

The software is configurable to support different charge controller PCBs with STM32F072, low-power STM32L072/3 or most recent STM32G431 MCUs:

- [Libre Solar MPPT 12/24V 20A with CAN](https://github.com/LibreSolar/MPPT-2420-LC)
- [Libre Solar MPPT 12V 10A with USB](https://github.com/LibreSolar/MPPT-1210-HUS)
- [Libre Solar PWM 12/24V 20A](https://github.com/LibreSolar/PWM-2420-LUS)

## Building and flashing the firmware

This repository contains git submodules, so you need to clone (download) it by calling:

```
git clone --recursive https://github.com/LibreSolar/charge-controller-firmware
```

Unfortunately, the green GitHub "Clone or download" button does not include submodules. If you cloned the repository already and want to pull the submodules, run `git submodule update --init --recursive`.

### PlatfrormIO

It is suggested to use Visual Studio Code and PlatformIO for firmware development, as it simplifies compiling and uploading the code a lot:

1. Install Visual Studio Code and [PlatformIO](https://platformio.org/platformio-ide) to build the firmware.

2. Adjust configuration in `zephyr/prj.conf` if necessary.

3. Select the correct board in `platformio.ini` by removing the comment before the board name under `[platformio]` or create a file `custom.ini` with your personal settings.

4. Connect the board via a programmer. See the Libre Solar website for [further project-agnostic instructions](http://libre.solar/docs/flashing).

5. Press the upload button at the bottom left corner in VS Code.

### Native Zephyr environment

You can also use the Zephyr build system directly for advanced configuration using `menuconfig` or if you need more recently added features.

The CMake entry point is in the `zephyr` subfolder, so you need to run `west` command in that directory.

Initial board selection (see `boards subfolder for correct names):

        west build -b <board-name>

Flash with specific debug probe (runner), e.g. J-Link:

        west flash -r jlink

User configuration using menuconfig:

        west build -t menuconfig

Report of used memory (RAM and flash):

        west build -t rom_report
        west build -t ram_report

### Troubleshooting

#### Errors with STM32L072 MCU using OpenOCD

The standard OpenOCD settings sometimes fail to flash firmware to this MCU, which is used in the MPPT 1210 HUS and the PWM charge controller.

Try one of these workarounds:

1. Change OpenOCD settings to `adapter_khz 500` in line 70 in `~/.platformio/packages/tool-openocd/scripts/target/stm32l0.cfg`.

2. Use other tools or debug probes such as [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) or [Segger J-Link](https://www.segger.com/products/debug-probes/j-link/).

#### Connection failures

If flashing fails like this in PlatformIO:

```
Error: init mode failed (unable to connect to the target)
in procedure 'program'
** OpenOCD init failed **
shutdown command invoked
```

or with ST-Link:

```bash
$ st-flash write .pio/build/mppt-1210-hus-v0.4/firmware.bin 0x08000000
st-flash 1.5.1
2019-06-21T18:13:03 INFO common.c: Loading device parameters....
2019-06-21T18:13:03 WARN common.c: unknown chip id! 0x5fa0004
```

check the connection between the programmer (for example the ST-Link of the Nucleo board) and the charge controller.

## Bootloader Support (STM32L07x only)

The custom linker script file STM32L073XZ.ld.link_script.ld needs to be updated before generating the application firmware binary. For each application, the flash start address and the maximum code size need to be updated in this file. Currently, the locations 0x08001000 and 0x08018000 are used for applications 1 & 2 respectively.

## API documentation

The documentation auto-generated by Doxygen can be found [here](https://libre.solar/charge-controller-firmware/).

## Conformance testing

We are using Travis CI to perform several checks for each commit or pull-request. In order to run
the same tests locally (recommended before each pull-request) use the script `check.sh` or ` check.bat`:

    bash check.sh       # Linux
    cmd check.bat       # Windows

### Unit-tests

In order to run the unit tests, you need a PlatformIO Plus account. Run tests with the following command:

    platformio test -e unit-test-native

### Static code analysis

PlatformIO integrates cppcheck and clangtidy. The following command is an example to run the checks for MPPT 1210 HUS charge controller code:

    platformio check -e mppt-1210-hus-v0.7

## Additional firmware documentation (docs folder)

- [MPPT charger firmware details](docs/firmware.md)
- [Charger state machine](docs/charger.md)
