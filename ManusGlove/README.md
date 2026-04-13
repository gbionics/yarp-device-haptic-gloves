# Dependencies to install and use the Manus Glove Device

# Manus Glove SDK  

## Windows 
Download the `SDK` and `MANUS Core 2 Online` from https://docs.manus-meta.com/latest/Resources/. The device is tested with the `2.5.1` version.
Create a folder named `"ManusGlove-API"`, and extract the files inside the folder.

To install the device, it is highly suggested to do it via the [robotology-superbuild](https://github.com/robotology/robotology-superbuild).
Before doing so, make sure to export the following environment variables:
```bash
set ManusGlove_DIR=C:\ManusGlove-API\SDKClient\SDKClient_Windows
set PATH=%PATH%;%ManusGlove_DIR%\ManusSDK\lib
```


## Linux
Download the `SDK` from https://docs.manus-meta.com/latest/Resources/. The device is tested only with Manus SDK **Version v3.** (with the `Metagloves Pro`) and Ubuntu 22.04. However, `ManusCore` is not available: instead, we launch `ManusIntegrated`.

For the dependencies, you can refer to https://docs.manus-meta.com/latest/Plugins/SDK/Linux/.
Summarizing, we only need to install the dependencies for Manus `Integrated`, not `Remote`. Launch:

```sh
sudo apt-get update && sudo apt-get install -y build-essential git libtool libzmq3-dev libusb-1.0-0-dev zlib1g-dev libudev-dev gdb libncurses5-dev && sudo apt-get clean
```

To allow connections to MANUS hardware you need to place the following file in `/etc/udev/rules.d/70-manus-hid-rules`.
So, run `sudo nano /etc/udev/rules.d/70-manus-hid-rules` and put:

```
# HIDAPI/libusb
SUBSYSTEMS=="usb", ATTRS{idVendor}=="3325", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="83fd", MODE:="0666"

# HIDAPI/hidraw
KERNEL=="hidraw*", ATTRS{idVendor}=="3325", MODE:="0666"
```


Before using the glove, it is highly advisable to perform calibration. In order to do so, you need to build the examples.
Move to 

```
cd <PATH_TO>/SDKClient_Linux
make all LDFLAGS="-L./ManusSDK/lib -lManusSDK_integrated -lncurses -lpthread -Wl,-rpath=./ManusSDK/lib"
```
Now you can run:
```
./SDKClient_Linux.out 
```

### Install the device

Export the environmental variables:

```bash
export ManusGlove_DIR=<PATH_TO>/ManusSDK_v3.1.1/SDKClient_Linux
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ManusGlove_DIR$/ManusSDK/lib
```

Run `cmake`, make sure to set the right `CMAKE_INSTALL_PREFIX`. If you are using the `superbuild`, point to the superbuild install prefix. 
If you are using a conda environment, you can use: 
```bash
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DYARP_DEVICE_MANUS_GLOVE_ENABLE=ON
```

If you are not using the `superbuild`, build and install:

```bash
make -j
make install
```

### Run the device
After installation, the `xml` should now be installed in the correct `YARP_DATA_DIRS`.
You can launch:
```bash
yarprobotinterface --config ManusGlove.xml
```



