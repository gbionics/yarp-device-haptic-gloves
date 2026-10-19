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

The default downloaded version is `3.1.1`.
If you want a different version, add:

```bash
-DMANUS_SDK_DOWNLOAD_VERSION=<version>
```

Examples:

```bash
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

## Generate the hand URDF

[`code/generate_hand_urdf.py`](code/generate_hand_urdf.py) builds a URDF of the user's hand from the MANUS raw-skeleton data, so that all joint angles equal to zero reproduce the captured pose. It works in two steps: a live capture from MANUS Core, which also saves the raw data to a JSON file, and an optional offline regeneration of the URDF from that JSON file.

### 1. Live capture (creates the JSON)

Live capture (`--live`) requires a running MANUS Core with the gloves connected and the MANUS Python SDK, i.e. the `Python` folder shipped with the MANUS Core SDK (tested with version `3.2.1`). From the repository root, using the Pixi environment (which provides `cffi`):

```bash
pixi run python ManusGlove/code/generate_hand_urdf.py --side right --live --manus-sdk-dir <PATH_TO>/MANUS_Core_<version>_SDK_Windows/Python --out-dir <OUTPUT_DIR>
```

Instead of `--manus-sdk-dir` you can set the `MANUS_SDK_PYTHON_DIR` environment variable. Use `--side left`, `--side right` or `--side both` (default), and `--remote` to search for MANUS Core on the network instead of localhost. Keep the hand still while the script waits for the data.

For each captured hand, the script writes two files in `--out-dir` (default: current directory):
- `<side>_hand.urdf`: the URDF of the hand;
- `<side>_raw_skeleton.json`: the raw-skeleton frame received from MANUS Core (one entry per node, with its parent, chain type, joint type, position and rotation).

#### Capturing from a MANUS Core recording (`.mrec`)

The capture can also be taken from a recording saved in MANUS Core (`.mrec`) instead of from the gloves worn live. Open the recording in MANUS Core and play it back, either in loop or paused on the frame you want to use for the URDF, then run the same `--live` command shown above. The script takes the first frame it receives, so pausing on the desired frame is the way to choose the captured pose.

In this case the gloves are not needed, but `--live` is still required (the data come from MANUS Core) and the MANUS dongle must be connected to the PC.

### 2. Regenerate the URDF from a saved capture (optional)

The JSON written in step 1 can be used to rebuild the URDF later, e.g. after changing the script, without MANUS Core, the gloves or the MANUS SDK. Only one hand at a time is supported:

```bash
pixi run python ManusGlove/code/generate_hand_urdf.py --side right --from-json <OUTPUT_DIR>/right_raw_skeleton.json --out-dir <OUTPUT_DIR>
```

## Generate the hand URDF in the FBX skeleton reference pose

[`code/generate_hand_urdf_from_fbx.py`](code/generate_hand_urdf_from_fbx.py) builds one URDF per hand from an FBX exported by MANUS Core. Instead of the pose captured live, the URDF uses the **reference pose (bind pose) of the skeleton stored in the FBX**, so joint angles equal to zero reproduce that pose regardless of the pose held during the recording. The FBX data are used as they are.

In the FBX export MANUS Core applies the recorded data to a target skeleton chosen in the [export settings](https://docs.manus-meta.com/3.2.0/Software/Rec/Exporting%20fbx/). Export with:
- **Skeleton: MANUS Polygon Avatar** (the only skeleton supported by the script);
- **Scale skeleton** enabled, otherwise the phalanx lengths are those of the avatar instead of the user's.

With these settings:
- phalanx lengths are the user's (MANUS calibration);
- metacarpals, palm and pose (finger bends, spread, thumb orientation) are those of the avatar skeleton, scaled to the user.

The live URDF (`generate_hand_urdf.py`) instead uses the MANUS raw skeleton for the whole hand (the same data exported in the Core CSV).

Link and joint names, axes, sign conventions and limits are the same as in `generate_hand_urdf.py`. No MANUS Core, gloves or SDK are needed:

```bash
pixi run python ManusGlove/code/generate_hand_urdf_from_fbx.py <FILE>.fbx --out-dir <OUTPUT_DIR>
```

It writes `left_hand_rest_pose.urdf` and `right_hand_rest_pose.urdf` in `--out-dir` (default: current directory); use `--side left` or `--side right` for a single hand. The script warns if the bone values differ from the FBX bind pose or use FBX features it ignores (pivots, offsets, scaling).

## Inspect a MANUS FBX export

[`code/read_fbx.py`](code/read_fbx.py) reads a binary FBX file (e.g. exported from a MANUS Core recording) and prints what it contains: file info (axis system, units, frame rate, objects, mesh), bone hierarchy with the static local transforms, bind/rest poses and animated channels. It only needs the Python standard library:

```bash
pixi run python ManusGlove/code/read_fbx.py <FILE>.fbx
```

Use `--info`, `--hierarchy`, `--poses` or `--animation` to print only some sections, and `--bones <REGEX>` to select bones (e.g. `--bones "_r$"` for the right hand). `--csv <FILE>.csv` exports the animated local transforms of the selected bones, one row per frame.

Note that a MANUS FBX contains the **retargeted** skeleton (the target skeleton chosen in the export), not the MANUS raw skeleton: its rotations are not the ergonomics angles. If "Use first frame bindpose" was enabled in the export, frame 0 is the skeleton bind pose, not recorded data.



