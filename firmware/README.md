# Firmware

Improvements from the [earlier firmware version](https://github.com/Kaiote-opensource/OpenHAP) include: 

1. Code rewritten in C++ from C improving code maintainability and abstraction.
2. Addition and separation of hardware validation, normalization/calibration and field measurement code.

##  Building The Firmware

The firmware build process has a dependency on the [ESP-IDF SDK](https://github.com/espressif/esp-idf), that must be set up correctly on the build machine.

Using Docker is the easiest way to get the SDK setup with minimal hassle.

### 1. Installing Docker

Follow the instructions at https://docs.docker.com/install/, if it is not installed yet.

### 2. Cloning this repository

```console
foo@bar:~$ git clone https://github.com/Kiogora/OpenHAPv2.git && cd OpenHAPv2/firmware
```

### 3. Setting up the build container

The next step is to setup and start an ESP-IDF build container. This will download the image if not already present on the build machine.

Run:

```console
foo@bar/OpenHAPv2/firmware:~$ docker run --rm -v $PWD:/usr/src/app -w /usr/src/app -it espressif/idf:release-v4.3
```

Or setup a container using other SDK versions:

```console
foo@bar/OpenHAPv2/firmware:~$ docker run --rm -v $PWD:/usr/src/app -w /usr/src/app -it espressif/idf:<Your desired image tag>
```

The firmware has been built and tested with ESP-IDF release version 4.3. ESP-IDF image tags follow ESP-IDF tag conventions on Github.

For further information, do reference the [official docker documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-docker-image.html) by Espressif Systems.


### 4. Building

The above step starts a shell inside the container (user `root@8c91937edbca` in this case)

Run build command:

```console
root@8c91937edbca:~$ idf.py build
```

If you need to setup configurations prior to building using the Kconfig tool bundled with ESP-IDF use:

```console
root@8c91937edbca:~$ idf.py menuconfig build
```

### 5. Flashing

Commands which communicate with the board, such as idf.py flash and idf.py monitor will not work in the container unless the serial port is passed through into the container. However currently this is not possible with Docker for Windows (https://github.com/docker/for-win/issues/1018) and Docker for Mac (https://github.com/docker/for-mac/issues/900).

A standard workaround across all OS's is to setup [esptool](https://github.com/espressif/esptool) and flash directly from the host OS.
