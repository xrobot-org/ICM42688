# ICM42688

TDK ICM42688 六轴 IMU 传感器模块 / TDK ICM42688 6-axis IMU Driver

## 硬件需求 / Required Hardware

spi\_icm42688, icm42688\_cs, icm42688\_int, pwm\_icm42688\_heat, ramfs, database

## 构造参数 / Constructor Arguments

* datarate:           ICM42688::DataRate::DATA\_RATE\_1KHZ
* accl\_range:         ICM42688::AcclRange::RANGE\_16G
* gyro\_range:         ICM42688::GyroRange::DPS\_2000
* rotation:           {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
* pid\_param:          {k: 0.2, p: 1.0, i: 0.1, d: 0.0, i\_limit: 0.3, out\_limit: 1.0, cycle: false}
* gyro\_topic\_name:    "icm42688\_gyro"
* accl\_topic\_name:    "icm42688\_accl"
* target\_temperature: 45.0
* task\_stack\_depth:   512

## 依赖 / Depends

* 无 （No dependencies）

## 滤波器配置 / Filter Configuration

![Group Delay](./Group%20Delay.png)

![Frequency Response](./Frequency%20Response.png)

