# SlimSerialRTDE

A general serial driver with frame decoding  in c++ and python, on platform of (Windows/Linux/Arm)

## Getting started
 

1. clone this repo  
```bash
cd ~

git clone  http://192.168.192.168:1080/chenxiaojiao/SlimSerialRTDE.git

```

3. build and install
```bash
cd ~/SlimSerialRTDE

sudo ./install.sh
```


# Python Support
> **_NOTE:_** if mypy is not installed , run the following to install
```
python3 -m pip install mypy
```
1. build and install python binding
```bash

pip install .
```

    

## Usage
### CMake project
1. in your CMakeList.txt, add
```cmake
find_package(SlimSerialRTDE)
```

2. include header in your project
```c++
#include <SlimSerialRTDE.h>
```

### In a python project

```python
import pySlimSerialRTDE as pysd
 
a=pysd.MonosSlimSerialClient()

a.connect("/dev/ttyUSB0")

#start arm valve
a.commandStart()

#arm command
a.commandPosition(1,0, 1,0,1,0,1,0,1)
a.commandPitchUp()
a.commandElongateMin()
a.commandGunLock(-1)

#stop arm valve
a.commandSop()

```

see file pythonExample.py
