#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: TDK ICM42688 6 轴 IMU（SPI）驱动模块 / Driver Module for the TDK ICM42688 6-axis IMU over SPI
depends: []
=== END MANIFEST === */
// clang-format on

#include <array>
#include <memory>

#include "database.hpp"
#include "gpio.hpp"
#include "libxr_def.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "ramfs.hpp"
#include "spi.hpp"
#include "thread.hpp"
#include "transform.hpp"

/**
 * @brief ICM42688 6 轴 IMU 驱动模块，负责初始化、数据采集、温控与 Topic 发布。
 *        Driver Module for the ICM42688 6-axis IMU: initialization, data acquisition,
 *        temperature control and Topic publishing.
 */
class ICM42688
{
 public:
  /// 角度到弧度的换算系数 (rad/deg)
  /// Degree-to-radian factor (rad/deg)
  static constexpr float M_DEG2RAD_MULT = 0.01745329251f;
  /// 数据区起始寄存器 TEMP_DATA1
  /// Start register of the data block, TEMP_DATA1
  static constexpr uint8_t ICM42688_REG_TEMP_DATA1 = 0x1D;
  /// 一次 burst 读取的字节数：温度 2 + 加速度 6 + 角速度 6
  /// Bytes per burst read: temperature 2 + acceleration 6 + angular velocity 6
  static constexpr uint8_t ICM42688_READ_LEN = 14;

  /**
   * @brief 陀螺仪与加速度计的 ODR，数值对应 GYRO_CONFIG0 / ACCEL_CONFIG0 的 ODR 字段。
   *        ODR of the gyroscope and the accelerometer; the value is the ODR field of
   *        GYRO_CONFIG0 / ACCEL_CONFIG0.
   */
  typedef enum : uint8_t
  {
    DATA_RATE_UNKNOW = 0,   ///< 未指定
                            ///< Unspecified
    DATA_RATE_32KHZ = 1,    ///< 32 kHz
    DATA_RATE_16KHZ = 2,    ///< 16 kHz
    DATA_RATE_8KHZ = 3,     ///< 8 kHz
    DATA_RATE_4KHZ = 4,     ///< 4 kHz
    DATA_RATE_2KHZ = 5,     ///< 2 kHz
    DATA_RATE_1KHZ = 6,     ///< 1 kHz
    DATA_RATE_200HZ = 7,    ///< 200 Hz
    DATA_RATE_100HZ = 8,    ///< 100 Hz
    DATA_RATE_50HZ = 9,     ///< 50 Hz
    DATA_RATE_25HZ = 10,    ///< 25 Hz
    DATA_RATE_12_5HZ = 11,  ///< 12.5 Hz
    DATA_RATE_500HZ = 15,   ///< 500 Hz
  } DataRate;

  /**
   * @brief 陀螺仪量程。
   *        Gyroscope range.
   */
  typedef enum : uint8_t
  {
    DPS_2000 = 0,    ///< ±2000 dps
    DPS_1000 = 1,    ///< ±1000 dps
    DPS_500 = 2,     ///< ±500 dps
    DPS_250 = 3,     ///< ±250 dps
    DPS_125 = 4,     ///< ±125 dps
    DPS_62_5 = 5,    ///< ±62.5 dps
    DPS_31_25 = 6,   ///< ±31.25 dps
    DPS_15_625 = 7,  ///< ±15.625 dps
  } GyroRange;

  /**
   * @brief 加速度计量程。
   *        Accelerometer range.
   */
  typedef enum : uint8_t
  {
    RANGE_16G = 0,  ///< ±16 g
    RANGE_8G = 1,   ///< ±8 g
    RANGE_4G = 2,   ///< ±4 g
    RANGE_2G = 3,   ///< ±2 g
  } AcclRange;

  /**
   * @brief ICM42688 配置参数。
   *        ICM42688 configuration parameters.
   */
  struct Param
  {
    DataRate data_rate;  ///< 陀螺仪与加速度计的 ODR
    ///< ODR of the gyroscope and the accelerometer
    AcclRange accl_range;  ///< 加速度计量程
    ///< Accelerometer range
    GyroRange gyro_range;  ///< 陀螺仪量程
    ///< Gyroscope range
    LibXR::Quaternion<float> rotation;  ///< 传感器到应用坐标系的四元数 (w, x, y, z)
    ///< Quaternion (w, x, y, z), sensor to application frame
    LibXR::PID<float>::Param pid_param;  ///< 温控 PID，输出为 PWM 占空比 (0.0-1.0)
    ///< Temperature PID, output is the PWM duty cycle (0.0-1.0)
    bool enable_clk_in;  ///< 为 true 时使用外部时钟输入 CLKIN
    ///< When true, use the external clock input CLKIN
    const char* gyro_topic_name;  ///< 陀螺仪 Topic 名称
    ///< Gyroscope Topic name
    const char* accl_topic_name;  ///< 加速度计 Topic 名称
    ///< Accelerometer Topic name
    float target_temperature;  ///< 目标温度 (°C)
    ///< Target temperature (°C)
    size_t task_stack_depth;  ///< 采样线程栈深
    ///< Sampling thread stack depth
  };

  /**
   * @brief 构造 ICM42688：注册中断与 RamFS 命令，初始化芯片，创建采样线程与温控任务。
   *        Construct ICM42688: register the interrupt and the RamFS command, initialize
   *        the chip, and create the sampling thread and the temperature-control task.
   *
   * @param cs 片选 GPIO。
   *           Chip-select GPIO.
   * @param interrupt INT1 数据就绪中断 GPIO，需由 BSP 配置为下降沿中断。
   *                  INT1 data-ready interrupt GPIO, configured by the BSP as a
   *                  falling-edge interrupt.
   * @param spi 连接 ICM42688 的 SPI。
   *            SPI connected to the ICM42688.
   * @param heater_pwm 加热电阻的 PWM。
   *                   PWM of the heating resistor.
   * @param database 保存陀螺仪零偏的 Database。
   *                 Database that stores the gyroscope zero offset.
   * @param ramfs 接收 `icm42688` 命令的 RamFS。
   *              RamFS that receives the `icm42688` command.
   * @param param 配置参数。
   *              Configuration parameters.
   */
  ICM42688(
      LibXR::GPIO& cs,
      LibXR::GPIO& interrupt,
      LibXR::SPI& spi,
      LibXR::PWM& heater_pwm,
      LibXR::Database& database,
      LibXR::RamFS& ramfs,
      const Param& param = {.data_rate = ICM42688::DataRate::DATA_RATE_1KHZ, .accl_range = ICM42688::AcclRange::RANGE_16G, .gyro_range = ICM42688::GyroRange::DPS_2000, .rotation = {1.0f, 0.0f, 0.0f, 0.0f}, .pid_param = {.k = 0.2f, .p = 1.0f, .i = 0.1f, .d = 0.0f, .i_limit = 0.3f, .out_limit = 1.0f, .cycle = false}, .enable_clk_in = false, .gyro_topic_name = "icm42688_gyro", .accl_topic_name = "icm42688_accl", .target_temperature = 45.0f, .task_stack_depth = 512})
      : data_rate_(param.data_rate),
        accl_range_(param.accl_range),
        gyro_range_(param.gyro_range),
        target_temperature_(param.target_temperature),
        enable_clk_in_(param.enable_clk_in),
        topic_gyro_(LibXR::Topic::CreateTopic<decltype(gyro_data_)>(param.gyro_topic_name)),
        topic_accl_(LibXR::Topic::CreateTopic<decltype(accl_data_)>(param.accl_topic_name)),
        cs_(std::addressof(cs)),
        int_(std::addressof(interrupt)),
        spi_(std::addressof(spi)),
        pwm_(std::addressof(heater_pwm)),
        rotation_(std::move(param.rotation)),
        pid_heat_(param.pid_param),
        op_spi_(sem_spi_),
        cmd_file_(LibXR::RamFS::CreateFile("icm42688", CommandFunc, this)),
        gyro_data_key_(database, "icm42688_gyro_data",
                       Eigen::Matrix<float, 3, 1>(0.0f, 0.0f, 0.0f))
  {
    ramfs.Add(cmd_file_);

    int_->DisableInterrupt();

    auto int_cb = LibXR::GPIO::Callback::Create(
        [](bool in_isr, ICM42688* self)
        {
          auto now = LibXR::Timebase::GetMicroseconds();
          self->dt_ = now - self->last_int_time_;
          self->last_int_time_ = now;
          self->new_data_.PostFromCallback(in_isr);
        },
        this);
    int_->RegisterCallback(int_cb);

    while (!Init())
    {
      XR_LOG_ERROR("ICM42688: Init failed. Retry...");
      LibXR::Thread::Sleep(100);
    }
    XR_LOG_PASS("ICM42688: Init success.");

    thread_.Create(this, ThreadFunc, "icm42688_thread", param.task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);

    void (*temp_ctrl_fun)(ICM42688*) = [](ICM42688* self)
    {
      float duty =
          self->pid_heat_.Calculate(self->target_temperature_, self->temperature_, 0.01f);
      duty = std::clamp(duty, 0.0f, 1.0f);
      self->pwm_->SetDutyCycle(duty);
    };

    auto temp_ctrl = LibXR::Timer::CreateTask(temp_ctrl_fun, this, 10);

    LibXR::Timer::Add(temp_ctrl);
    LibXR::Timer::Start(temp_ctrl);
  }

  /**
   * @brief 关闭加速度计与陀螺仪。
   *        Power off the accelerometer and the gyroscope.
   */
  void Off() { WriteSingle(0X4E, 0x00); }

  /**
   * @brief 以低噪声模式开启加速度计与陀螺仪。
   *        Power on the accelerometer and the gyroscope in low-noise mode.
   */
  void On() { WriteSingle(0X4E, 0x0f); }

  /**
   * @brief 软复位芯片，校验 WHO_AM_I，并配置滤波器、中断、量程与 ODR。
   *        Soft-reset the chip, check WHO_AM_I, and configure the filters, the
   *        interrupt, the ranges and the ODR.
   *
   * @return 初始化成功返回 true，WHO_AM_I 不符时返回 false。
   *         True on success, false when WHO_AM_I does not match.
   */
  bool Init()
  {
    /* Select Bank 0 */
    WriteSingle(0x76, 0x00);
    /* Software reset */
    WriteSingle(0x11, 0x01);
    LibXR::Thread::Sleep(5);
    /* Read INT status to switch SPI mode */
    auto buf = ReadSingle(0x2D);
    UNUSED(buf);
    /* Select Bank 0 */
    WriteSingle(0x76, 0x00);
    /* Check WhoAmI register */
    buf = ReadSingle(0x75);
    while (buf != 0x47)
    {
      return false;
    }

    Off();

    /***** Anti-Aliasing Filter Configuration *****/

    /* Configure GYRO anti-aliasing filters */
    /* Select Bank 1 */
    WriteSingle(0x76, 0x01);
    WriteSingle(0x0B, 0xA0);  // Enable anti-aliasing and notch filters
    WriteSingle(0x0C, 0x05);  // GYRO_AAF_DELT = 5 (default 13)
    WriteSingle(0x0D, 0x19);  // GYRO_AAF_DELTSQR = 25 (default 170)
    WriteSingle(0x0E, 0xa0);  // GYRO_AAF_BITSHIFT = 10 (default 8)

    /* Configure ACCEL anti-aliasing filters */
    /* Select Bank 2 */
    WriteSingle(0x76, 0x02);
    WriteSingle(0x03, 0x05);  // ACCEL_AAF_DELT = 5 (default 24)
    WriteSingle(0x04, 0x19);  // ACCEL_AAF_DELTSQR = 25 (default 64)
    WriteSingle(0x05, 0xa0);  // ACCEL_AAF_BITSHIFT = 10 (default 6)

    /***** Custom Filter Settings *****/

    /* Select Bank 0 */
    WriteSingle(0x76, 0x00);
    /* Interrupt output configuration */
    WriteSingle(0x14, 0x12);  // INT1 & INT2: pulse mode, active low
    /* Temp & Gyro_Config1 */
    WriteSingle(0x51, 0xca);  // Latency = 32ms, GYRO_UI_FILT_ORD=3
    /* GYRO_ACCEL_CONFIG0 */
    WriteSingle(0x52, 0x22);  // Set LPF bandwidth
    /* ACCEL_CONFIG1 */
    WriteSingle(0x53, 0x0D);  // Reserved / no config
    /* INT_CONFIG0 */
    WriteSingle(0x63, 0x00);  // Default
    /* INT_CONFIG1 */
    WriteSingle(0x64, 0x00);  // Enable interrupt pins
    /* INT_SOURCE0 */
    WriteSingle(0x65, 0x08);  // DRDY routed to INT1
    /* INT_SOURCE1 */
    WriteSingle(0x66, 0x00);  // Default
    /* INT_SOURCE3 */
    WriteSingle(0x68, 0x00);  // Default
    WriteSingle(0x69, 0x00);  // Default

    /* Disable AFSR (see: https://github.com/ArduPilot/ardupilot/pull/25332) */
    uint8_t intf = ReadSingle(0x4D);
    intf &= ~0xC0;
    intf |= 0x40;
    WriteSingle(0x4D, intf);

    /* Select Bank 0 */
    WriteSingle(0x76, 0x00);
    /* Power on sensors */
    On();

    /* Gyroscope configuration */
    WriteSingle(0x4F, (uint8_t(gyro_range_) << 5) | uint8_t(data_rate_));
    /* Accelerometer configuration */
    WriteSingle(0x50, (uint8_t(accl_range_) << 5) | uint8_t(data_rate_));

    /* Select Bank 0 */
    WriteSingle(0x76, 0x00);

    /* Enable RTC */
    WriteSingle(0x77, 0x95);

    /* Select Bank 1 */
    WriteSingle(0x76, 0x01);

    /* Enable external clock (CLKIN) */
    if (enable_clk_in_)
    {
      WriteSingle(0x7B, 0x04);
    }

    /* Select Bank 0 */
    WriteSingle(0x76, 0x00);

    LibXR::Thread::Sleep(50);
    int_->EnableInterrupt();

    return true;
  }

  /**
   * @brief 采样线程：启动加热 PWM，等待 data-ready 中断，读取并发布陀螺仪与加速度数据。
   *        Sampling thread: start the heater PWM, wait for the data-ready interrupt, then
   *        read and publish the gyroscope and acceleration data.
   *
   * @param self ICM42688 实例。
   *             ICM42688 instance.
   */
  static void ThreadFunc(ICM42688* self)
  {
    self->pwm_->SetConfig({30000});
    self->pwm_->SetDutyCycle(0);
    self->pwm_->Enable();

    while (true)
    {
      if (self->new_data_.Wait(50) == LibXR::ErrorCode::OK)
      {
        self->Read(ICM42688_REG_TEMP_DATA1, ICM42688_READ_LEN);
        self->Parse();
        self->topic_gyro_.Publish(self->gyro_data_);
        self->topic_accl_.Publish(self->accl_data_);
      }
    }
  }

  /**
   * @brief 写一个寄存器。
   *        Write one register.
   *
   * @param reg 寄存器地址。
   *            Register address.
   * @param data 写入的值。
   *             Value to write.
   */
  void WriteSingle(uint8_t reg, uint8_t data)
  {
    cs_->Write(false);
    spi_->MemWrite(reg, data, op_spi_);
    cs_->Write(true);
  }

  /**
   * @brief 等待 50 ms 后读一个寄存器。
   *        Wait 50 ms, then read one register.
   *
   * @param reg 寄存器地址。
   *            Register address.
   * @return 寄存器的值。
   *         Register value.
   */
  uint8_t ReadSingle(uint8_t reg)
  {
    LibXR::Thread::Sleep(50);
    uint8_t data = 0;
    cs_->Write(false);
    spi_->MemRead(reg, data, op_spi_);
    cs_->Write(true);
    return data;
  }

  /**
   * @brief 从起始寄存器连续读取 len 字节到内部缓冲区。
   *        Read len bytes from the start register into the internal buffer.
   *
   * @param reg 起始寄存器地址。
   *            Start register address.
   * @param len 读取字节数。
   *            Number of bytes to read.
   */
  void Read(uint8_t reg, uint8_t len)
  {
    cs_->Write(false);
    spi_->MemRead(reg, {buffer_, len}, op_spi_);
    cs_->Write(true);
  }

  /**
   * @brief 解析缓冲区中的温度、加速度与角速度；角速度减去零偏，两者再乘以 rotation。
   *        Parse the temperature, acceleration and angular velocity in the buffer; the
   *        angular velocity has the zero offset subtracted, and both vectors are
   *        multiplied by rotation.
   */
  void Parse()
  {
    int16_t t = static_cast<int16_t>(buffer_[0] << 8 | buffer_[1]);
    temperature_ = static_cast<float>(t) / 132.48f + 25.0f;

    std::array<int16_t, 3> accl_raw_u16, gyro_raw_u16;
    std::array<float, 3> accl_raw, gyro_raw;

    for (int i = 0; i < 3; i++)
    {
      accl_raw_u16[i] =
          static_cast<int16_t>(buffer_[i * 2 + 2] << 8 | buffer_[i * 2 + 3]);
      accl_raw[i] = static_cast<float>(accl_raw_u16[i]) * GetAcclLSB();

      gyro_raw_u16[i] =
          static_cast<int16_t>(buffer_[i * 2 + 8] << 8 | buffer_[i * 2 + 9]);
      gyro_raw[i] = static_cast<float>(gyro_raw_u16[i]) * GetGyroLSB() * M_DEG2RAD_MULT;
    }

    if (in_cali_)
    {
      gyro_cali_.data()[0] += gyro_raw_u16[0];
      gyro_cali_.data()[1] += gyro_raw_u16[1];
      gyro_cali_.data()[2] += gyro_raw_u16[2];
      cali_counter_++;
    }

    accl_data_ =
        rotation_ * Eigen::Matrix<float, 3, 1>(accl_raw[0], accl_raw[1], accl_raw[2]);

    gyro_data_ = rotation_ *
                 Eigen::Matrix<float, 3, 1>(
                     Eigen::Matrix<float, 3, 1>(gyro_raw[0], gyro_raw[1], gyro_raw[2]) -
                     gyro_data_key_.data_);
  }

  /**
   * @brief 监控回调：数据含 NaN 或 Inf 时输出警告；中断间隔偏离 ODR 理想周期超过
   *        150 us 时输出频率错误警告。
   *        Monitor callback: log a warning when the data contains NaN or Inf, and a
   *        frequency-error warning when the interrupt interval deviates from the ideal
   *        ODR period by more than 150 us.
   */
  void OnMonitor(void)
  {
    if (std::isinf(gyro_data_.x()) || std::isinf(gyro_data_.y()) ||
        std::isinf(gyro_data_.z()) || std::isinf(accl_data_.x()) ||
        std::isinf(accl_data_.y()) || std::isinf(accl_data_.z()) ||
        std::isnan(gyro_data_.x()) || std::isnan(gyro_data_.y()) ||
        std::isnan(gyro_data_.z()) || std::isnan(accl_data_.x()) ||
        std::isnan(accl_data_.y()) || std::isnan(accl_data_.z()))
    {
      XR_LOG_WARN("ICM42688: NaN data detected. gyro: %f %f %f, accl: %f %f %f",
                  gyro_data_.x(), gyro_data_.y(), gyro_data_.z(), accl_data_.x(),
                  accl_data_.y(), accl_data_.z());
    }

    float ideal_dt = 0.0f;

    switch (data_rate_)
    {
      case DataRate::DATA_RATE_32KHZ:
        ideal_dt = 0.00003125f;
        break;
      case DataRate::DATA_RATE_16KHZ:
        ideal_dt = 0.0000625f;
        break;
      case DataRate::DATA_RATE_8KHZ:
        ideal_dt = 0.000125f;
        break;
      case DataRate::DATA_RATE_4KHZ:
        ideal_dt = 0.00025f;
        break;
      case DataRate::DATA_RATE_2KHZ:
        ideal_dt = 0.0005f;
        break;
      case DataRate::DATA_RATE_1KHZ:
        ideal_dt = 0.001f;
        break;
      case DataRate::DATA_RATE_500HZ:
        ideal_dt = 0.002f;
        break;
      case DataRate::DATA_RATE_200HZ:
        ideal_dt = 0.005f;
        break;
      case DataRate::DATA_RATE_100HZ:
        ideal_dt = 0.01f;
        break;
      case DataRate::DATA_RATE_50HZ:
        ideal_dt = 0.02f;
        break;
      case DataRate::DATA_RATE_25HZ:
        ideal_dt = 0.04f;
        break;
      case DataRate::DATA_RATE_12_5HZ:
        ideal_dt = 0.08f;
        break;
      default:
        XR_LOG_ERROR("Unknown data rate.");
        break;
    }
    if (std::fabs(dt_.ToSecondf() - ideal_dt) > 0.00015f)
    {
      XR_LOG_WARN("ICM42688 Frequency Error: %6f", dt_.ToSecondf());
    }
  }

  /**
   * @brief RamFS 命令 `icm42688`：显示数据、查看陀螺仪零偏、校准零偏。
   *        RamFS command `icm42688`: show data, list the gyroscope zero offset, and
   *        calibrate the zero offset.
   *
   * @param self ICM42688 实例。
   *             ICM42688 instance.
   * @param argc 参数个数。
   *             Argument count.
   * @param argv 参数列表。
   *             Argument list.
   * @return 0 表示命令已处理；参数个数无效时返回 -1。
   *         0 when the command is handled; -1 when the argument count is invalid.
   */
  static int CommandFunc(ICM42688* self, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<
          "  show [time_ms] [interval_ms] - Print sensor data "
          "periodically.\r\n">();
      LibXR::STDIO::Printf<
          "  list_offset                  - Show current gyro calibration "
          "offset.\r\n">();
      LibXR::STDIO::Printf<
          "  cali                         - Start gyroscope "
          "calibration.\r\n">();
    }
    else if (argc == 2)
    {
      if (strcmp(argv[1], "list_offset") == 0)
      {
        LibXR::STDIO::Printf<"Current calibration offset - x: %f, y: %f, z: %f\r\n">(
            self->gyro_data_key_.data_.x(), self->gyro_data_key_.data_.y(),
            self->gyro_data_key_.data_.z());
      }
      else if (strcmp(argv[1], "cali") == 0)
      {
        self->gyro_data_key_.data_.x() = 0.0, self->gyro_data_key_.data_.y() = 0.0,
        self->gyro_data_key_.data_.z() = 0.0;
        self->gyro_cali_ = Eigen::Matrix<int64_t, 3, 1>(0.0, 0.0, 0.0);
        self->cali_counter_ = 0;
        self->in_cali_ = true;
        LibXR::STDIO::Printf<
            "Starting gyroscope calibration. Please keep the device "
            "steady.\r\n">();
        LibXR::Thread::Sleep(3000);
        for (int i = 0; i < 60; i++)
        {
          LibXR::STDIO::Printf<"Progress: %d / 60\r">(i);
          LibXR::Thread::Sleep(1000);
        }
        LibXR::STDIO::Printf<"\r\nProgress: Done\r\n">();
        self->in_cali_ = false;
        LibXR::Thread::Sleep(1000);

        self->gyro_data_key_.data_.x() = static_cast<double>(self->gyro_cali_.data()[0]) /
                                         static_cast<double>(self->cali_counter_) *
                                         self->GetGyroLSB() * M_DEG2RAD_MULT;
        self->gyro_data_key_.data_.y() = static_cast<double>(self->gyro_cali_.data()[1]) /
                                         static_cast<double>(self->cali_counter_) *
                                         self->GetGyroLSB() * M_DEG2RAD_MULT;
        self->gyro_data_key_.data_.z() = static_cast<double>(self->gyro_cali_.data()[2]) /
                                         static_cast<double>(self->cali_counter_) *
                                         self->GetGyroLSB() * M_DEG2RAD_MULT;

        LibXR::STDIO::Printf<"\r\nCalibration result - x: %f, y: %f, z: %f\r\n">(
            self->gyro_data_key_.data_.x(), self->gyro_data_key_.data_.y(),
            self->gyro_data_key_.data_.z());

        LibXR::STDIO::Printf<"Analyzing calibration quality...\r\n">();
        self->gyro_cali_ = Eigen::Matrix<int64_t, 3, 1>(0.0, 0.0, 0.0);
        self->cali_counter_ = 0;
        self->in_cali_ = true;
        for (int i = 0; i < 60; i++)
        {
          LibXR::STDIO::Printf<"Progress: %d / 60\r">(i);
          LibXR::Thread::Sleep(1000);
        }
        LibXR::STDIO::Printf<"\r\nProgress: Done\r\n">();
        self->in_cali_ = false;
        LibXR::Thread::Sleep(1000);

        LibXR::STDIO::Printf<"\r\nCalibration error - x: %f, y: %f, z: %f\r\n">(
            static_cast<double>(self->gyro_cali_.data()[0]) /
                    static_cast<double>(self->cali_counter_) * self->GetGyroLSB() *
                    M_DEG2RAD_MULT -
                self->gyro_data_key_.data_.x(),
            static_cast<double>(self->gyro_cali_.data()[1]) /
                    static_cast<double>(self->cali_counter_) * self->GetGyroLSB() *
                    M_DEG2RAD_MULT -
                self->gyro_data_key_.data_.y(),
            static_cast<double>(self->gyro_cali_.data()[2]) /
                    static_cast<double>(self->cali_counter_) * self->GetGyroLSB() *
                    M_DEG2RAD_MULT -
                self->gyro_data_key_.data_.z());

        self->gyro_data_key_.Set(self->gyro_data_key_.data_);
        LibXR::STDIO::Printf<"Calibration data saved.\r\n">();
      }
    }
    else if (argc == 4)
    {
      if (strcmp(argv[1], "show") == 0)
      {
        int time = std::atoi(argv[2]);
        int delay = std::atoi(argv[3]);

        delay = std::clamp(delay, 2, 1000);

        while (time > 0)
        {
          LibXR::STDIO::Printf<
              "Accel: x = %+5f, y = %+5f, z = %+5f | "
              "Gyro: x = %+5f, y = %+5f, z = %+5f | Temp: %+5f\r\n">(
              self->accl_data_.x(), self->accl_data_.y(), self->accl_data_.z(),
              self->gyro_data_.x(), self->gyro_data_.y(), self->gyro_data_.z(),
              self->temperature_);
          LibXR::Thread::Sleep(delay);
          time -= delay;
        }
      }
    }
    else
    {
      LibXR::STDIO::Printf<"Error: Invalid arguments.\r\n">();
      return -1;
    }
    return 0;
  }

  /**
   * @brief 当前加速度计量程下一个 LSB 对应的加速度。
   *        Acceleration represented by one LSB at the current accelerometer range.
   *
   * @return 单位 g/LSB。
   *         Value in g/LSB.
   */
  float GetAcclLSB()
  {
    switch (accl_range_)
    {
      case AcclRange::RANGE_16G:
        return 1.0 / 2048.0;
      case AcclRange::RANGE_8G:
        return 1.0 / 4096.0;
      case AcclRange::RANGE_4G:
        return 1.0 / 8192.0;
      case AcclRange::RANGE_2G:
        return 1.0 / 16384.0;
      default:
        ASSERT(false);
        return 0.0;
    }
  }

  /**
   * @brief 当前陀螺仪量程下一个 LSB 对应的角速度。
   *        Angular velocity represented by one LSB at the current gyroscope range.
   *
   * @return 单位 dps/LSB。
   *         Value in dps/LSB.
   */
  float GetGyroLSB()
  {
    switch (gyro_range_)
    {
      case GyroRange::DPS_2000:
        return 1.0 / 16.384f;
      case GyroRange::DPS_1000:
        return 1.0 / 32.768f;
      case GyroRange::DPS_500:
        return 1.0 / 65.536f;
      case GyroRange::DPS_250:
        return 1.0 / 131.072f;
      case GyroRange::DPS_125:
        return 1.0 / 262.144f;
      case GyroRange::DPS_62_5:
        return 1.0 / 524.288f;
      case GyroRange::DPS_31_25:
        return 1.0 / 1048.576f;
      case GyroRange::DPS_15_625:
        return 1.0 / 2097.152f;
      default:
        ASSERT(false);
        return 0.0;
    }
  }

 private:
  DataRate data_rate_;
  AcclRange accl_range_;
  GyroRange gyro_range_;
  float temperature_ = 0.0f;
  float target_temperature_ = 25.0f;

  bool enable_clk_in_ = false;
  bool in_cali_ = false;
  uint32_t cali_counter_ = 0;
  Eigen::Matrix<std::int64_t, 3, 1> gyro_cali_;

  LibXR::MicrosecondTimestamp last_int_time_ = 0;
  LibXR::MicrosecondTimestamp::Duration dt_ = 0;

  uint8_t buffer_[ICM42688_READ_LEN];
  Eigen::Matrix<float, 3, 1> gyro_data_, accl_data_;

  LibXR::Topic topic_gyro_, topic_accl_;
  LibXR::GPIO *cs_, *int_;
  LibXR::SPI* spi_;
  LibXR::PWM* pwm_;
  LibXR::Quaternion<float> rotation_;
  LibXR::PID<float> pid_heat_;
  LibXR::Semaphore sem_spi_, new_data_;
  LibXR::SPI::OperationRW op_spi_;
  LibXR::RamFS::File cmd_file_;
  LibXR::Database::Key<Eigen::Matrix<float, 3, 1>> gyro_data_key_;
  LibXR::Thread thread_;
};
