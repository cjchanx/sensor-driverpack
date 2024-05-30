# MPL3115A2S Altimeter Driver

## Setup
1. Add this repository as a submodule, or download the MPL3115A2S folder
2. From the root directory of this repo, add I2CPeripheral_Interface.h and write a new .c file that contains the implementation for I2C_Write and I2C_Read (use the provided I2CPeripheral_STM32HAL as a reference)
3. Include "MPL3115A2S_Driver.h" in the file that needs to call driver functions
4. Define three configuration structs: `MPL3115A2S_Config cfg{0}; MPL3115A2S_Data_Config dcfg {0}; MPL3115A2S_Int_Config icfg {0};`
5. Configure the structs based on the [datasheet](https://www.mouser.ca/datasheet/2/302/MPL3115A2S-3103208.pdf) or the [Usage Examples](#usage-example) below
6. Depending on your configuration, run the relevant functions to read. See the [documentation](#documentation) or [examples](#usage-example) below.

## Documentation
The [header file](MPL3115A2S_Driver.h) contains a comment above each function declaration describing parameters and return values
### API at a Glance
```C++
MPL_OP_STATUS MPL3115A2S_Init(MPL3115A2S_Config* const cfg, MPL3115A2S_Data_Config* const data_cfg, MPL3115A2S_Int_Config* const int_cfg);

MPL_OP_STATUS MPL3115A2S_ReadDataPolling(float* pres_alt, float* temp);
MPL_OP_STATUS MPL3115A2S_ReadDataExtInterrupt(float* pres_alt, float* temp);

MPL_OP_STATUS MPL3115A2S_ReadPressure(float* pressure);
MPL_OP_STATUS MPL3115A2S_ReadAltitude(float* altitude);
MPL_OP_STATUS MPL3115A2S_ReadTemperature(float* temperature);
MPL_OP_STATUS MPL3115A2S_ReadPressureInt(uint32_t* pressure, uint16_t multiplier);
MPL_OP_STATUS MPL3115A2S_ReadAltitudeInt(int32_t* altitude, uint16_t multiplier);
MPL_OP_STATUS MPL3115A2S_ReadTemperatureInt(int32_t* temperature, uint16_t multiplier);

MPL_OP_STATUS MPL3115A2S_Reset();
MPL_OP_STATUS MPL3115A2S_CheckDeviceID();
MPL_OP_STATUS MPL3115A2S_DataReady();
MPL_OP_STATUS MPL3115A2S_ReadIntSource(uint8_t* int_source);
MPL_OP_STATUS MPL3115A2S_DataReadyIT();
MPL_OP_STATUS MPL3115A2S_IsAltimeterMode();
```

## Usage Example

### Polling Mode (built-in data ready check)
```C++
  MPL3115A2S_Config cfg {0};
  MPL3115A2S_Data_Config dcfg {0};
  MPL3115A2S_Int_Config icfg {0};

  cfg.altimeter_mode = 1;
  cfg.os_ratio = MPL3115A2S_CTRL_OS32;
  dcfg.enable_all_flags = 1;

  MPL3115A2S_Init(&cfg, &dcfg, &icfg);

  float alt, temp;
  MPL3115A2S_ReadDataPolling(&alt, &temp);
```

### Interrupt Mode
```C++
  MPL3115A2S_Config cfg {0};
  MPL3115A2S_Data_Config dcfg {0};
  MPL3115A2S_Int_Config icfg {0};

  cfg.altimeter_mode = 1;
  cfg.os_ratio = MPL3115A2S_CTRL_OS32;
  dcfg.enable_all_flags = 1;

  icfg.int1_active_high = 0; // INT1 - Active low
  icfg.int1_open_drain = 1;  // INT1 - Open drain
  icfg.int2_active_high = 0; // INT2 - Active low
  icfg.int2_open_drain = 1;  // INT2 - Open drain
  icfg.data_ready_it_enable = 1; // Enable Data Ready Interrupt

  MPL3115A2S_Init(&cfg, &dcfg, &icfg);

  // -> Assume ISR triggers, causing the following code to run
  // (by a semaphore, queue event, flag, etc. to avoid blocking in ISR)
  float alt, temp;
  MPL3115A2S_ReadDataExtInterrupt(&alt, &temp);
```


### Polling Mode (manual/no data ready check)
```C++
  MPL3115A2S_Config cfg {0};
  MPL3115A2S_Data_Config dcfg {0};
  MPL3115A2S_Int_Config icfg {0};

  cfg.altimeter_mode = 1;
  cfg.os_ratio = MPL3115A2S_CTRL_OS32;
  dcfg.enable_all_flags = 1;

  MPL3115A2S_Init(&cfg, &dcfg, &icfg);

  float alt;
  MPL3115A2S_ReadAltitude(&alt);

  float temp;
  MPL3115A2S_ReadTemperature(&temp);
```

### 
