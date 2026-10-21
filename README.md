# ``yarp-device-haptic-gloves``

This repository contains the YARP device drivers for the following haptic gloves:
- [Manus Glove](ManusGlove): enabled with the CMake option `YARP_DEVICE_MANUS_GLOVE_ENABLE`
- [SenseGlove](SenseGlove): enabled with the CMake option `YARP_DEVICE_SENSE_GLOVE_ENABLE`
- [Weart Glove](WeartGlove): enabled with the CMake option `YARP_DEVICE_WEART_GLOVE_ENABLE`

## Dependencies
1. [``YARP``](https://github.com/robotology/yarp)
2. [``human-dynamics-estimation``](https://github.com/robotology/human-dynamics-estimation)
3. [``Eigen3``](https://eigen.tuxfamily.org/) 

Additional dependencies are specified in the README of each glove in the corresponding folder.

## Development environment with Pixi

The root `pixi.toml` defines the dependencies used by the Manus CI workflow. Install them with:

```bash
pixi install
```

Run commands in the environment with `pixi run <command>`, or activate it with `pixi shell`.
