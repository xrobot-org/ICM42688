# ICM42688

TDK ICM42688 6 轴 IMU（SPI）驱动模块 / Driver Module for the TDK ICM42688 6-axis IMU over SPI

## 1. 模块作用 / Purpose

构造时，ICM42688 软复位芯片，校验 `WHO_AM_I` 为 `0x47`，配置陀螺仪与加速度计的抗混叠滤波器（DELT 5、DELTSQR 25、BITSHIFT 10）、UI 滤波器和 INT1（脉冲、低电平有效，data-ready 映射到 INT1），关闭 AFSR，再按 `Param` 写入量程和 ODR。校验失败时每 100 ms 重试。中断 GPIO `interrupt` 由 BSP 配置为下降沿中断输入。

采样线程 `icm42688_thread`（REALTIME 优先级）等待 data-ready 中断，用一次 burst 读出温度、加速度和角速度并发布。加速度单位为 g，乘以 `rotation`；角速度单位为 rad/s，先减去零偏再乘以 `rotation`。温度为 `raw / 132.48 + 25`（°C）。

加热 PWM 以 30 kHz 运行；一个周期 10 ms 的 LibXR 定时器任务用 `pid_param` 计算占空比，限制在 0 到 1，使温度趋近 `target_temperature`。

陀螺仪零偏（rad/s）保存在 Database 的键 `icm42688_gyro_data` 中。

`OnMonitor()` 在数据出现 NaN 或 Inf 时输出警告，并在中断间隔偏离 ODR 对应的理想周期超过 150 µs 时输出 `ICM42688 Frequency Error`。

模块在 RamFS 中注册命令 `icm42688`：

- `icm42688`：打印用法。
- `icm42688 show <time_ms> <interval_ms>`：在 `time_ms` 内每隔 `interval_ms`（限制在 2 到 1000 ms）打印一次加速度、角速度和温度。
- `icm42688 list_offset`：打印当前陀螺仪零偏。
- `icm42688 cali`：陀螺仪零偏校准，期间设备保持静止。先等待 3 s，再采集 60 s 求平均零偏，然后采集 60 s 打印残差，最后把零偏写入 Database。

Upon construction, ICM42688 soft-resets the chip, checks that `WHO_AM_I` is `0x47`, configures the anti-aliasing filters of the gyroscope and the accelerometer (DELT 5, DELTSQR 25, BITSHIFT 10), the UI filters and INT1 (pulse, active low, data-ready routed to INT1), disables AFSR, and then writes the ranges and the ODR from `Param`. When the check fails, it retries every 100 ms. The interrupt GPIO `interrupt` is configured by the BSP as a falling-edge interrupt input.

The sampling thread `icm42688_thread` (REALTIME priority) waits for the data-ready interrupt, reads the temperature, acceleration and angular velocity in one burst and publishes them. The acceleration unit is g, multiplied by `rotation`; the angular velocity unit is rad/s, with the zero offset subtracted before the multiplication by `rotation`. The temperature is `raw / 132.48 + 25` (°C).

The heater PWM runs at 30 kHz; a LibXR timer task with a 10 ms period computes the duty cycle with `pid_param`, limits it to 0 to 1, and drives the temperature toward `target_temperature`.

The gyroscope zero offset (rad/s) is stored in the Database under the key `icm42688_gyro_data`.

`OnMonitor()` logs a warning when the data contains NaN or Inf, and logs `ICM42688 Frequency Error` when the interrupt interval deviates from the ideal period of the ODR by more than 150 µs.

The Module registers the command `icm42688` in RamFS:

- `icm42688`: print the usage.
- `icm42688 show <time_ms> <interval_ms>`: print the acceleration, angular velocity and temperature every `interval_ms` (limited to 2 to 1000 ms) for `time_ms`.
- `icm42688 list_offset`: print the current gyroscope zero offset.
- `icm42688 cali`: gyroscope zero-offset calibration, with the device held still. It waits 3 s, collects 60 s to average the zero offset, collects another 60 s to print the residual, and finally writes the zero offset to the Database.

## 2. 滤波器特性 / Filter Response

陀螺仪的抗混叠滤波器为二阶，3 dB 带宽 213 Hz；UI 滤波器为三阶，带宽设置为 max(400 Hz, ODR)/5。默认 ODR 1 kHz 时，UI 滤波器的 3 dB 带宽为 195.8 Hz，直流群延迟为 2.7 ms。以上数值取自 ICM-42688-P 数据手册（DS-000347）第 5.3 节和第 5.5 节的表格，其他 ODR 下的数值见同一表格。

The gyroscope anti-alias filter is second order with a 3 dB bandwidth of 213 Hz; the UI filter is third order with its bandwidth set to max(400 Hz, ODR)/5. At the default ODR of 1 kHz, the UI filter has a 3 dB bandwidth of 195.8 Hz and a group delay of 2.7 ms at DC. These values come from the tables in sections 5.3 and 5.5 of the ICM-42688-P datasheet (DS-000347), which also list the values for the other ODRs.

## 3. 构造接口 / Constructor

```cpp
ICM42688(LibXR::GPIO& cs,
         LibXR::GPIO& interrupt,
         LibXR::SPI& spi,
         LibXR::PWM& heater_pwm,
         LibXR::Database& database,
         LibXR::RamFS& ramfs,
         const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `cs`：片选 GPIO（输出，低有效），取自 BSP 的硬件注册（`XR_REGISTER`）。
- `interrupt`：INT1 数据就绪中断 GPIO，由 BSP 配置为下降沿中断。
- `spi`：连接 ICM42688 的 `LibXR::SPI`。
- `heater_pwm`：IMU 加热电阻的 `LibXR::PWM`。
- `database`：保存陀螺仪零偏的 `LibXR::Database`。
- `ramfs`：注册 `icm42688` 命令的 `LibXR::RamFS`。

配置参数（`Param`）：

- `data_rate`：陀螺仪与加速度计的 ODR，默认 `DATA_RATE_1KHZ`；可选 32 kHz 到 12.5 Hz 的各档以及 `DATA_RATE_500HZ`。
- `accl_range`：加速度计量程，默认 `RANGE_16G`；可选 16、8、4、2 g。
- `gyro_range`：陀螺仪量程，默认 `DPS_2000`；可选 2000 dps 到 15.625 dps。
- `rotation`：传感器坐标系到应用坐标系的四元数 `{w, x, y, z}`，默认单位四元数。
- `pid_param`：温控 PID，`LibXR::PID<float>::Param`，字段为 `k, p, i, d, i_limit, out_limit, cycle`，默认 `k = 0.2`、`p = 1.0`、`i = 0.1`、`d = 0`、`i_limit = 0.3`、`out_limit = 1.0`、`cycle = false`。
- `enable_clk_in`：为 `true` 时使用外部时钟输入 CLKIN，默认 `false`。
- `gyro_topic_name`、`accl_topic_name`：发布的 Topic 名称，默认 `"icm42688_gyro"`、`"icm42688_accl"`。
- `target_temperature`：目标温度，单位 °C，默认 45。
- `task_stack_depth`：采样线程栈深，单位字节，默认 512。

Dependencies:

- `cs`: chip-select GPIO (output, active low), taken from the BSP's Registration (`XR_REGISTER`).
- `interrupt`: INT1 data-ready interrupt GPIO, configured by the BSP as a falling-edge interrupt.
- `spi`: the `LibXR::SPI` connected to the ICM42688.
- `heater_pwm`: the `LibXR::PWM` of the IMU heating resistor.
- `database`: the `LibXR::Database` that stores the gyroscope zero offset.
- `ramfs`: the `LibXR::RamFS` that receives the `icm42688` command.

Configuration parameters (`Param`):

- `data_rate`: ODR of the gyroscope and the accelerometer, default `DATA_RATE_1KHZ`; options are the steps from 32 kHz down to 12.5 Hz and `DATA_RATE_500HZ`.
- `accl_range`: accelerometer range, default `RANGE_16G`; options are 16, 8, 4 and 2 g.
- `gyro_range`: gyroscope range, default `DPS_2000`; options are 2000 dps down to 15.625 dps.
- `rotation`: quaternion `{w, x, y, z}` from the sensor frame to the application frame, default identity.
- `pid_param`: temperature-control PID, `LibXR::PID<float>::Param` with fields `k, p, i, d, i_limit, out_limit, cycle`, default `k = 0.2`, `p = 1.0`, `i = 0.1`, `d = 0`, `i_limit = 0.3`, `out_limit = 1.0`, `cycle = false`.
- `enable_clk_in`: when `true`, the external clock input CLKIN is used, default `false`.
- `gyro_topic_name`, `accl_topic_name`: names of the published Topics, default `"icm42688_gyro"` and `"icm42688_accl"`.
- `target_temperature`: target temperature in °C, default 45.
- `task_stack_depth`: stack depth of the sampling thread in bytes, default 512.

## 4. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `gyro_topic_name`（默认 `icm42688_gyro`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 角速度，单位 rad/s，已去零偏并旋转 |
| `accl_topic_name`（默认 `icm42688_accl`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 加速度，单位 g，已旋转 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `gyro_topic_name` (default `icm42688_gyro`) | Publish | `Eigen::Matrix<float, 3, 1>` | Angular velocity in rad/s, zero offset removed and rotated |
| `accl_topic_name` (default `icm42688_accl`) | Publish | `Eigen::Matrix<float, 3, 1>` | Acceleration in g, rotated |

## 5. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/ICM42688` 写入的实例，依赖填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/ICM42688`, with the dependencies set to names registered by the BSP with `XR_REGISTER` (Registration):

```yaml
modules:
  - module: xrobot-org/ICM42688
    id: icm42688_0
    args:
      - cs: icm42688_cs
      - interrupt: icm42688_int
      - spi: spi1
      - heater_pwm: imu_heat_pwm
      - database: database
      - ramfs: ramfs
      - param:
          data_rate: ICM42688::DataRate::DATA_RATE_1KHZ
          accl_range: ICM42688::AcclRange::RANGE_16G
          gyro_range: ICM42688::GyroRange::DPS_2000
          rotation: '{1.0f, 0.0f, 0.0f, 0.0f}'
          pid_param:
            k: 0.2f
            p: 1.0f
            i: 0.1f
            d: 0.0f
            i_limit: 0.3f
            out_limit: 1.0f
            cycle: false
          enable_clk_in: false
          gyro_topic_name: "icm42688_gyro"
          accl_topic_name: "icm42688_accl"
          target_temperature: 45.0f
          task_stack_depth: 512
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一片通过 SPI 连接的 ICM42688，带片选 GPIO 与 INT1 中断 GPIO，另有一路驱动加热电阻的 PWM；SPI、GPIO、PWM、Database 与 RamFS 由 BSP 通过 `XR_REGISTER` 注册。

Dependencies: LibXR.

Hardware: one ICM42688 connected over SPI, with a chip-select GPIO and an INT1 interrupt GPIO, and one PWM output that drives the heating resistor; the SPI, GPIOs, PWM, Database and RamFS are registered by the BSP with `XR_REGISTER`.
