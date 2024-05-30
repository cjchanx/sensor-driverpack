# sensor-driverpack

Collection of various sensor drivers compatible with C/C++ projects, intended to support a variety of microcontrollers.

![C](https://img.shields.io/badge/c-%2300599C.svg?style=for-the-badge&logo=c&logoColor=white)
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![Badge Name](https://img.shields.io/badge/NXP-%23fca908.svg?style=for-the-badge&logo=<badge>&logoColor=<logo-color>)
![STMicro](https://user-images.githubusercontent.com/78698227/185344511-0296b5ed-15a3-4013-a98a-6dcd38222382.svg)


![](https://img.shields.io/github/repo-size/cjchanx/sensor-driverpack?label=Size)
![](https://img.shields.io/github/commit-activity/m/cjchanx/sensor-driverpack)
![](https://img.shields.io/github/contributors/cjchanx/sensor-driverpack)

## Usage Instructions

Navigate to the relevent sensor's folder and read the README. Each sensor will require a communication interface. For example the MPL3115A2S altimeter requires the [I2CPeripheral_Interface.h](I2CPeripheral_Interface.h) file which contains two function declarations that the user must define for the particular microcontroller in any new `.c` file, a basic example using STM32's HAL can be found in [I2CPeripheral_STM32HAL.c](I2CPeripheral_STM32HAL.c).

## Sensors
### [MPL3115A2S](MPL3115A2S) NXP Pressure Sensor with Altimetry Support [(Datasheet)](https://www.mouser.ca/datasheet/2/302/MPL3115A2S-3103208.pdf)
- I2C interface, currently supports various configurations for polling and interrupt modes.
