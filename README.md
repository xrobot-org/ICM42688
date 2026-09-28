# ICM42688

TDK ICM42688 六轴 IMU 传感器模块。
TDK ICM42688 6-axis IMU driver.

## 行为 / Behaviour

- 构造时软复位芯片，校验 `WHO_AM_I` 为 `0x47`，配置陀螺仪与加速度计的抗混叠滤波器
  （DELT 5、DELTSQR 25、BITSHIFT 10）、UI 滤波器、INT1（脉冲、低电平有效，data-ready 映射到
  INT1）、关闭 AFSR，再按 `Param` 写入量程和 ODR。校验失败时每 100 ms 重试。
  The constructor soft-resets the chip, checks `WHO_AM_I` = `0x47`, configures the
  gyroscope and accelerometer anti-aliasing filters (DELT 5, DELTSQR 25, BITSHIFT
  10), the UI filters and INT1 (pulse, active low, data-ready routed to INT1),
  disables AFSR, then writes ranges and ODR from `Param`. A failed check retries
  every 100 ms.
- 模块不配置中断 GPIO 的方向和触发沿，由 BSP 把 `interrupt` 配置为下降沿中断输入。
  The Module does not configure the interrupt GPIO direction or edge; the BSP must
  set `interrupt` up as a falling-edge interrupt input.
- 采样线程 `icm42688_thread`（REALTIME 优先级）等待 data-ready 中断，一次读出温度、加速度和
  角速度并发布。/ The `icm42688_thread` thread (REALTIME priority) waits for the
  data-ready interrupt, reads temperature, acceleration and angular rate in one
  burst and publishes them.
- 加速度单位 g，乘以 `rotation`；角速度单位 rad/s，先减去零偏再乘以 `rotation`。温度为
  `raw / 132.48 + 25` °C。/ Acceleration in g, multiplied by `rotation`; angular rate
  in rad/s, offset subtracted before the rotation. Temperature is
  `raw / 132.48 + 25` °C.
- 加热 PWM 以 30 kHz 运行；一个 10 ms 周期的 LibXR 定时器任务用 `pid_param` 计算占空比并
  限制在 0-1，使温度趋近 `target_temperature`。/ The heater PWM runs at 30 kHz; a 10 ms
  LibXR timer task computes the duty cycle from `pid_param`, clamped to 0-1, to hold
  `target_temperature`.
- 陀螺仪零偏（rad/s）保存在 Database 键 `icm42688_gyro_data`。/ The gyroscope offset
  (rad/s) is stored in the Database key `icm42688_gyro_data`.
- `OnMonitor()` 在数据出现 NaN/Inf 时告警，并在中断间隔偏离 ODR 理想周期超过 150 µs 时打印
  `ICM42688 Frequency Error`。/ `OnMonitor()` warns on NaN/Inf data and prints
  `ICM42688 Frequency Error` when the interrupt interval deviates from the ODR period
  by more than 150 µs.

## Topic

| Topic | 类型 / Type | 内容 / Content |
| --- | --- | --- |
| `gyro_topic_name`（默认 / default `icm42688_gyro`） | `Eigen::Matrix<float, 3, 1>` | 角速度 / angular rate, rad/s |
| `accl_topic_name`（默认 / default `icm42688_accl`） | `Eigen::Matrix<float, 3, 1>` | 加速度 / acceleration, g |

## RamFS 命令 / RamFS command

- `icm42688`：打印用法。/ Print usage.
- `icm42688 show <time_ms> <interval_ms>`：每 `interval_ms`（2-1000 ms）打印一次加速度、
  角速度和温度，持续 `time_ms`。/ Print acceleration, angular rate and temperature
  every `interval_ms` (2-1000 ms) for `time_ms`.
- `icm42688 list_offset`：打印当前陀螺仪零偏。/ Print the current gyroscope offset.
- `icm42688 cali`：陀螺仪零偏校准。设备需保持静止：等待 3 s 后采集 60 s 求平均零偏，再采集
  60 s 打印残差，最后写入 Database。/ Gyroscope offset calibration. Keep the device
  still: after 3 s the offset is averaged over 60 s, the residual is measured over
  another 60 s, and the offset is saved to the Database.

## 依赖 / Dependencies

无其他模块依赖，仅使用 LibXR。
No other Modules; LibXR only.

## 构造接口 / Constructor

```cpp
ICM42688(LibXR::GPIO& cs,
         LibXR::GPIO& interrupt,
         LibXR::SPI& spi,
         LibXR::PWM& heater_pwm,
         LibXR::Database& database,
         LibXR::RamFS& ramfs,
         const Param& param = {...});
```

依赖 / Dependencies:

- `cs`：片选 GPIO（输出，低有效）。/ Chip-select GPIO (output, active low).
- `interrupt`：INT1 数据就绪中断 GPIO（由 BSP 配置为下降沿中断）。/ INT1 data-ready
  interrupt GPIO (configured by the BSP as falling-edge interrupt).
- `spi`：连接 ICM42688 的 `LibXR::SPI`。/ SPI bus of the ICM42688.
- `heater_pwm`：IMU 加热电阻的 `LibXR::PWM`。/ PWM of the IMU heater.
- `database`：保存陀螺仪零偏。/ Stores the gyroscope offset.
- `ramfs`：注册 `icm42688` 命令。/ Receives the `icm42688` command.

配置 / Configuration (`Param`，括号内为默认值 / defaults in brackets):

- `data_rate`：陀螺仪与加速度计 ODR（`DATA_RATE_1KHZ`），可选 32 kHz 到 12.5 Hz 以及
  `DATA_RATE_500HZ`。/ Gyroscope and accelerometer ODR, 32 kHz down to 12.5 Hz plus
  `DATA_RATE_500HZ`.
- `accl_range`：加速度计量程（`RANGE_16G`），可选 16 / 8 / 4 / 2 g。/ Accelerometer range.
- `gyro_range`：陀螺仪量程（`DPS_2000`），可选 2000 到 15.625 dps。/ Gyroscope range.
- `rotation`：传感器坐标系到应用坐标系的四元数 `{w, x, y, z}`（单位四元数）。
  / Quaternion from sensor frame to application frame (identity).
- `pid_param`：温控 PID 参数（`k = 0.2`、`p = 1.0`、`i = 0.1`、`d = 0`、`i_limit = 0.3`、
  `out_limit = 1.0`）。/ Heater PID parameters.
- `enable_clk_in`：使用外部时钟输入 CLKIN（`false`）。/ Use the external CLKIN clock input.
- `gyro_topic_name` / `accl_topic_name`：Topic 名称（`"icm42688_gyro"` / `"icm42688_accl"`）。
  / Topic names.
- `target_temperature`：目标温度，°C（45）。/ Target temperature in °C.
- `task_stack_depth`：采样线程栈深（512）。/ Stack depth of the sampling thread.

## 使用 / Use

```sh
xrobot module add xrobot-org/ICM42688
xrobot setup
xrobot instance add xrobot-org/ICM42688
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把依赖项填为 BSP 中用 `XR_REGISTER` 注册的对象名：
`xrobot instance add` writes an instance to `User/xrobot.yaml` with empty
dependencies and the source defaults; set the dependencies to the names of objects
the BSP registers with `XR_REGISTER`:

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
          rotation:
            - 1.0f
            - 0.0f
            - 0.0f
            - 0.0f
          pid_param:
            k: 0.2f
            p: 1.0f
            i: 0.1f
            d: 0.0f
            i_limit: 0.3f
            out_limit: 1.0f
            cycle: 'false'
          enable_clk_in: 'false'
          gyro_topic_name: '"icm42688_gyro"'
          accl_topic_name: '"icm42688_accl"'
          target_temperature: 45.0f
          task_stack_depth: '512'
```

BSP 侧 / BSP side:

```cpp
XR_REGISTER(icm42688_cs, LibXR::GPIO);
XR_REGISTER(icm42688_int, LibXR::GPIO);
XR_REGISTER(spi1, LibXR::SPI);
XR_REGISTER(imu_heat_pwm, LibXR::PWM);
XR_REGISTER(database, LibXR::Database);
XR_REGISTER(ramfs, LibXR::RamFS);
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。
Run `xrobot setup` again to generate `User/xrobot_main.hpp`.

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/xrobot-org/ICM42688`
（在 BSP 中）打印 manifest 和当前的构造函数。
`xrobot module show .` in this repository, or
`xrobot module show Modules/xrobot-org/ICM42688` in a BSP, prints the manifest and
the current constructor.

## 滤波器配置 / Filter Configuration

![Group Delay](./Group%20Delay.png)

![Frequency Response](./Frequency%20Response.png)
