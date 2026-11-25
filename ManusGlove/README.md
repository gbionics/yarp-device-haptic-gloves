# Dependencies to install and use the Manus Glove Device

# Manus Glove SDK  

## Windows 
Download the `SDK` and `MANUS Core 2 Online` from https://docs.manus-meta.com/latest/Resources/. The device is tested with the `2.5.1` version.
Create a folder named `"ManusGlove-API"`, and extract the files inside the folder.

Before building and installing the device, make sure to export the following environmental variables:
```
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

To allow connections to MANUS hardware you need to place the following file in `/etc/udev/rules.d/70-manus-hid-rules.rules`.
So, run `sudo nano /etc/udev/rules.d/70-manus-hid-rules.rules` and put:

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

When running the `cmake` commands below, make sure to set the right `CMAKE_INSTALL_PREFIX`. If you are using the `robotology-superbuild`, point it to the superbuild install prefix.
If you are using a conda environment, you can use $CONDA_PREFIX as the install prefix (as in the examples below).

You can build ManusGlove in two ways.

1. Automatic SDK download (recommended)


```bash
mkdir build && cd build
cmake .. \
	-DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX \
	-DYARP_DEVICE_MANUS_GLOVE_ENABLE=ON \
	-DENABLE_MANUS_SDK_DOWNLOAD=ON
```

The default downloaded version is `3.2.1`.
If you want a different version, add:

```bash
-DMANUS_SDK_DOWNLOAD_VERSION=<version>
```

Examples:

```bash
-DMANUS_SDK_DOWNLOAD_VERSION=3.2.1
-DMANUS_SDK_DOWNLOAD_VERSION=3.1.1
-DMANUS_SDK_DOWNLOAD_VERSION=2.5.1
```

If you downloaded the SDK during the build, the install step also places the ManusSDK runtime libraries in `<INSTALL_PREFIX>/lib`.


2. Manual SDK setup

If you prefer to provide the SDK yourself, export:

```bash
export ManusGlove_DIR=<PATH_TO>/ManusSDK_v3.1.1/SDKClient_Linux
```

```bash
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DYARP_DEVICE_MANUS_GLOVE_ENABLE=ON
```

If you are not using the `superbuild`, build and install:

```bash
make -j
make install
```

If you installed in a non-system prefix, export:

```bash
export YARP_DATA_DIRS=$YARP_DATA_DIRS:<INSTALL_PREFIX>/share/yarp
```

### Run the device
After installation, the device and the `yarprobotinterface` config `.xml` should now be installed in the correct `YARP_DATA_DIRS`.
You can launch:
```bash
yarprobotinterface --config ManusGlove.xml
```

### Parameters
The `ManusGlove` device accepts the following parameters:

| Parameter | Required | Default | Description |
|---|---|---|---|
| `period` | No | `0.01` | Period (in seconds) of the device thread reading the glove data. |
| `wearableName` | No | `ManusGlove` | Name of the wearable, used as prefix for the sensor names. |
| `is_right_hand` | Yes | - | `true` for the right hand glove, `false` for the left one. |
| `is_local_host` | Yes | - | If `true`, look for a Manus host only on the local machine, otherwise on the whole network. |
| `use_manus_core` | No | `true` | If `true`, connect to a running Manus Core; if `false`, use the Manus SDK in Integrated mode (no Manus Core needed, e.g. on Linux). |
| `glove_connection_timeout` | No | `5.0` | Maximum time (in seconds) to wait for the glove to be detected, otherwise the device fails to open. A value `<= 0` skips the check. |
| `publish_raw_data` | No | `false` | If `true`, also publish the raw glove joint values (in degrees, before coupling matrix and offset) as additional joint sensors with the `raw_deg` prefix. |
| `human_joint_list` | Yes | - | List of the human hand joint names to publish (e.g. `l_index_proximal`, `r_thumb_oppose`). |
| `human_finger_list` | Yes | - | List of the human finger names, used to publish a fingertip link sensor for each finger. |
| `hand_link` | Yes | - | Name of the human hand link, whose orientation is estimated by the glove IMU. |
| `min_joint_limit_deg` | Yes | - | Lower joint limits (in degrees), one per joint in `human_joint_list`. |
| `max_joint_limit_deg` | Yes | - | Upper joint limits (in degrees), one per joint in `human_joint_list`. |
| `coupling_matrix` | Yes | - | Row-major square matrix (size equal to the number of joints) applied to the raw joint values. |
| `offset_vector_deg` | Yes | - | Offset (in degrees) added to the joint values after applying the coupling matrix, one per joint. |
| `palm_frame_name` | No | `""` | Name of the palm frame used to publish the finger transforms through the attached `frameTransformClient`. If empty, no transform is published. |
| `palm_transform` | No | identity | Row-major 4x4 homogeneous transform applied to the finger transforms before publishing them in `palm_frame_name`. |

> [!NOTE]
> When opening the device, the process opening it blocks until a glove is detected or `glove_connection_timeout` expires, and it cannot be interrupted.


