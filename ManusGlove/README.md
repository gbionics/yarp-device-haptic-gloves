# Dependencies to install and use the Manus Glove Device

# Dependencies
### Manus Glove SDK  
- Download the SDK from https://www.manus-meta.com/resources/downloads/quantum-metagloves
- Extract the files into a folder and rename it "ManusGlove-API". The device works only with 2.5.1 version
- Add following environment variable:

```bash
set ManusGlove_DIR=C:\ManusGlove-API\SDKClient\SDKClient_Windows
set PATH=%PATH%;%ManusGlove_DIR%\ManusSDK\lib
```
### Manus Core
- Download and install the Manus Core from [here](https://www.manus-meta.com/resources/downloads/quantum-metagloves).

