# Firmware

Improvements from the [earlier firmware version](https://github.com/Kaiote-opensource/OpenHAP) include: 

1. Code rewritten in C++ from C improving code maintainability and abstraction.
2. Addition and separation of hardware validation, normalization/calibration and field measurement code.

##  Building The Firmware Locally

The firmware build process has a dependency on the [ESP-IDF SDK](https://github.com/espressif/esp-idf), that must be set up correctly on the build machine.

Using Docker is the easiest way to get the SDK setup with minimal hassle.

### 1. Installing Docker

Follow the instructions at https://docs.docker.com/install, if it is not installed yet.

### 2. Cloning this repository

```console
git clone https://github.com/Kiogora/OpenHAPv2.git && cd OpenHAPv2/firmware
```

### 3. Setting up the build container

The next step is to setup and start an ESP-IDF build container. This will download the image if not already present on the build machine.

The firmware folder containing the main.cpp file to build is passed as an environmental variable to the docker container. The folder options are:

- hardwareValidation
- particulateSensorNormalization

On linux distro host, run:

```console
docker run --env MAINFOLDER=hardwareValidation --rm -v $PWD:/usr/src/app -w /usr/src/app -it espressif/idf:release-v4.3
```
or
```console
docker run --env MAINFOLDER=particulateSensorNormalization --rm -v $PWD:/usr/src/app -w /usr/src/app -it espressif/idf:release-v4.3
```

On Windows powershell, run:

```console
docker run --env MAINFOLDER=hardwareValidation --rm -v ${PWD}:/usr/src/app -w /usr/src/app -it espressif/idf:release-v4.3
```
or
```console
docker run --env MAINFOLDER=particulateSensorNormalization --rm -v ${PWD}:/usr/src/app -w /usr/src/app -it espressif/idf:release-v4.3
```

On Windows command-prompt, run:

```console
docker run --env MAINFOLDER=hardwareValidation --rm -v %cd%:/usr/src/app -w /usr/src/app -it espressif/idf:release-v4.3
```
or
```console
docker run --env MAINFOLDER=particulateSensorNormalization --rm -v %cd%:/usr/src/app -w /usr/src/app -it espressif/idf:release-v4.3
```

The firmware has been built and tested with ESP-IDF release version 4.3. ESP-IDF image tags follow ESP-IDF tag conventions on Github. You may change the commands above to reflect your desired image.

For further information, do reference the [official docker documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-docker-image.html) by Espressif Systems.


### 4. Building

Run build command:

```console
idf.py build
```

Or if you need to setup firmware configurations prior to building using the Kconfig tool bundled with ESP-IDF, use:

```console
idf.py menuconfig build
```

For further information, see https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/#get-started-build

### 5. Flashing

Commands which communicate with the board, such as idf.py flash and idf.py monitor will not work in the container unless the serial port is passed through into the container. However currently this is not possible with Docker for Windows (https://github.com/docker/for-win/issues/1018) and Docker for Mac (https://github.com/docker/for-mac/issues/900).

A standard workaround across windows is to setup [esptool](https://github.com/espressif/esptool) and flash directly from the host OS.

#### - Exit from the container

```console
exit()
```

#### - Install esptool

```console
python -m pip install esptool
```

or on Windows

```console
python.exe -m pip install esptool
```
#### - Flash 

```console
python.exe -m pip esptool -p (PORT) -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/main.bin
```

or on Windows

```console
python.exe -m pip esptool -p (PORT) -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/main.bin
```
where PORT is the desired serial port port.
