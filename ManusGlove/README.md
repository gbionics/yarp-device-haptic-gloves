# Dependencies to install and use the Manus Glove Device

### Manus Glove SDK 
1. Inside `C:\` create a folder  called `ManusGlove-API`
2. Download Manus Core 2.5.1 SDK. The device works only with 2.5.1 version. 
3. Extract the files and place the folder called SDK_Client inside `C:\ManusGlove-API`
4. Clone https://github.com/gbionics/yarp-device-haptic-gloves by running the command below:

```bash
git clone https://github.com/gbionics/yarp-device-haptic-gloves 
```
5.  Add the following environment variable:

```bash
set ManusGlove_DIR=C:\ManusGlove-API\SDKClient\SDKClient_Windows
set PATH=%PATH%;%ManusGlove_DIR%\ManusSDK\lib
```
### Manus Core
1. Download Manus Core 2.5.1 Online Installer
2. Install Manus Core 2.5.1 following the instructions 
3. Restart the PC


### Build `yarp-device-haptic-gloves`

1. Create the build folder

```bash
cd C:\yarp-device-haptic-gloves
mkdir build
cd build
```

2. Configure the project with CMake

```bash
cmake -G "Visual Studio 17 2022" ..
ccmake .
```

3. Configure the required options

In the `ccmake` interface set the following variables:

* **CMAKE_INSTALL_PREFIX** → Set this variable to the directory where you want to install the device.
 e.g:
  ```
  C:/robotology-superbuild/build/install
  ```

* **YARP_DEVICE_MANUS_GLOVE_ENABLE** →

  ```
  ON
  ```

After setting the variables, press **`c`** to configure and **`g`** to generate the build files


4. Build and install

```bash
cmake --build . --config Release --target INSTALL
```


