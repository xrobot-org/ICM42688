#pragma once

// clang-format off
/* === MODULE MANIFEST ===
module_name: ICM42688
module_description: TDK ICM42688 六轴 IMU 传感器模块 / TDK ICM42688 6-axis IMU Driver
constructor_args:
  - rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
  - pid_param:
      k: 0.2
      p: 1.0
      i: 0.1
      d: 0.0
      i_limit: 0.3
      out_limit: 1.0
      cycle: false
  - gyro_topic_name: "icm42688_gyro"
  - accl_topic_name: "icm42688_accl"
  - target_temperature: 45.0
  - task_stack_depth: 512
required_hardware: spi_icm42688 icm42688_cs icm42688_int pwm_icm42688_heat ramfs database
repository: https://github.com/xrobot-org/ICM42688
=== END MANIFEST === */
// clang-format on

#include <array>

#include "app_framework.hpp"
#include "gpio.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "spi.hpp"
#include "transform.hpp"

template <typename HardwareContainer>
class ICM42688 : public LibXR::Application {
 public:
  static constexpr float DEG2RAD = 0.01745329251f;
  static constexpr uint8_t ICM42688_REG_TEMP_DATA1 = 0x1D;
  static constexpr uint8_t ICM42688_READ_LEN = 14;

  typedef enum {
    DATA_RATE_UNKNOW,
    DATA_RATE_32KHZ,
    DATA_RATE_16KHZ,
    DATA_RATE_8KHZ,
    DATA_RATE_4KHZ,
    DATA_RATE_2KHZ,
    DATA_RATE_1KHZ,
    DATA_RATE_200HZ,
    DATA_RATE_100HZ,
    DATA_RATE_50HZ,
    DATA_RATE_25HZ,
    DATA_RATE_12_5HZ,
    DATA_RATE_500HZ = 0XF,
  } DataRate;

  ICM42688(HardwareContainer &hw, LibXR::ApplicationManager &app,
           LibXR::Quaternion<float> &&rotation,
           LibXR::PID<float>::Param &&pid_param, const char *gyro_topic_name,
           const char *accl_topic_name, float target_temperature,
           size_t task_stack_depth)
      : target_temperature_(target_temperature),
        topic_gyro_(gyro_topic_name, sizeof(gyro_data_)),
        topic_accl_(accl_topic_name, sizeof(accl_data_)),
        cs_(hw.template FindOrExit<LibXR::GPIO>({"icm42688_cs"})),
        int_(hw.template FindOrExit<LibXR::GPIO>({"icm42688_int"})),
        spi_(hw.template FindOrExit<LibXR::SPI>({"spi_icm42688"})),
        pwm_(hw.template FindOrExit<LibXR::PWM>({"pwm_icm42688_heat"})),
        rotation_(std::move(rotation)),
        pid_heat_(pid_param),
        op_spi_(sem_spi_),
        cmd_file_(LibXR::RamFS::CreateFile("icm42688", CommandFunc, this)),
        gyro_data_key_(*hw.template FindOrExit<LibXR::Database>({"database"}),
                       "icm42688_gyro_data",
                       Eigen::Matrix<float, 3, 1>(0.0f, 0.0f, 0.0f)) {
    app.Register(*this);

    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    int_->DisableInterrupt();

    auto int_cb = LibXR::Callback<>::Create(
        [](bool in_isr, ICM42688<HardwareContainer> *self) {
          auto now = LibXR::Timebase::GetMicroseconds();
          self->dt_ = now - self->last_int_time_;
          self->last_int_time_ = now;
          self->new_data_.PostFromCallback(in_isr);
        },
        this);
    int_->RegisterCallback(int_cb);

    while (!Init()) {
      LibXR::STDIO::Printf("ICM42688: Init failed. Retry...\r\n");
      LibXR::Thread::Sleep(100);
    }
    LibXR::STDIO::Printf("ICM42688: Init success.\r\n");

    thread_.Create(this, ThreadFunc, "icm42688_thread", task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);

    void (*temp_ctrl_fun)(ICM42688<HardwareContainer> *) =
        [](ICM42688<HardwareContainer> *self) {
          float duty = self->pid_heat_.Calculate(self->target_temperature_,
                                                 self->temperature_, 0.01f);
          duty = std::clamp(duty, 0.0f, 1.0f);
          self->pwm_->SetDutyCycle(duty);
        };

    auto temp_ctrl = LibXR::Timer::CreateTask(temp_ctrl_fun, this, 10);

    LibXR::Timer::Add(temp_ctrl);
    LibXR::Timer::Start(temp_ctrl);
  }

  void Off() { WriteSingle(0X4E, 0x00); }
  void On() { WriteSingle(0X4E, 0x0f); }

  bool Init(DataRate data_rate = DATA_RATE_1KHZ) {
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
    while (buf != 0x47) {
      return false;
    }

    Off();

    /***** Anti-Aliasing Filter Configuration *****/

    /* Configure GYRO anti-aliasing filters */
    /* Select Bank 1 */
    WriteSingle(0x76, 0x01);
    WriteSingle(0x0B, 0xA0);  // Enable anti-aliasing and notch filters
    WriteSingle(0x0C, 0x0C);  // GYRO_AAF_DELT = 12 (default 13)
    WriteSingle(0x0D, 0x90);  // GYRO_AAF_DELTSQR = 144 (default 170)
    WriteSingle(0x0E, 0x80);  // GYRO_AAF_BITSHIFT = 8 (default 8)

    /* Configure ACCEL anti-aliasing filters */
    /* Select Bank 2 */
    WriteSingle(0x76, 0x02);
    WriteSingle(0x03, 0x18);  // ACCEL_AAF_DELT = 12 (default 24)
    WriteSingle(0x04, 0x90);  // ACCEL_AAF_DELTSQR = 144 (default 64)
    WriteSingle(0x05, 0x80);  // ACCEL_AAF_BITSHIFT = 8 (default 6)

    /***** Custom Filter Set #1 @227.2Hz *****/

    /* Select Bank 0 */
    WriteSingle(0x76, 0x00);
    /* Filter settings */
    WriteSingle(0x52, 0x22);  // Set to 227.2Hz (code 0x03)

    /* Interrupt output configuration */
    WriteSingle(0x14, 0x12);  // INT1 & INT2: pulse mode, active low
    /* Temp & Gyro_Config1 */
    WriteSingle(0x51, 0x96);  // Latency = 8ms
    /* GYRO_ACCEL_CONFIG0 */
    WriteSingle(0x52, 0x44);  // Set 1-bandwidth mode
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
    WriteSingle(0x4F, data_rate | 0x40);  // 500dps, 1kHz
    /* Accelerometer configuration */
    WriteSingle(0x50, data_rate | 0x40);  // 4G, 1kHz

    /* Select Bank 0 */
    WriteSingle(0x76, 0x00);

    /* Enable RTC */
    WriteSingle(0x77, 0x95);

    /* Select Bank 1 */
    WriteSingle(0x76, 0x01);

    /* Enable external clock (CLKIN) */
    WriteSingle(0x7B, 0x04);

    /* Select Bank 0 */
    WriteSingle(0x76, 0x00);

    LibXR::Thread::Sleep(50);
    int_->EnableInterrupt();

    return true;
  }

  static void ThreadFunc(ICM42688<HardwareContainer> *self) {
    self->pwm_->SetConfig({30000});
    self->pwm_->SetDutyCycle(0);
    self->pwm_->Enable();

    while (true) {
      if (self->new_data_.Wait(50) == ErrorCode::OK) {
        self->Read(ICM42688_REG_TEMP_DATA1, ICM42688_READ_LEN);
        self->Parse();
        self->topic_gyro_.Publish(self->gyro_data_);
        self->topic_accl_.Publish(self->accl_data_);
      }
    }
  }

  void WriteSingle(uint8_t reg, uint8_t data) {
    cs_->Write(false);
    spi_->MemWrite(reg, data, op_spi_);
    cs_->Write(true);
  }

  uint8_t ReadSingle(uint8_t reg) {
    LibXR::Thread::Sleep(50);
    uint8_t data = 0;
    cs_->Write(false);
    spi_->MemRead(reg, data, op_spi_);
    cs_->Write(true);
    return data;
  }

  void Read(uint8_t reg, uint8_t len) {
    cs_->Write(false);
    spi_->MemRead(reg, {buffer_, len}, op_spi_);
    cs_->Write(true);
  }

  void Parse() {
    int16_t t = static_cast<int16_t>(buffer_[0] << 8 | buffer_[1]);
    temperature_ = static_cast<float>(t) / 132.48f + 25.0f;

    std::array<int16_t, 3> accl_raw_u16, gyro_raw_u16;
    std::array<float, 3> accl_raw, gyro_raw;

    for (int i = 0; i < 3; i++) {
      accl_raw_u16[i] =
          static_cast<int16_t>(buffer_[i * 2 + 2] << 8 | buffer_[i * 2 + 3]);
      accl_raw[i] = static_cast<float>(accl_raw_u16[i]) / 8192.0f;

      gyro_raw_u16[i] =
          static_cast<int16_t>(buffer_[i * 2 + 8] << 8 | buffer_[i * 2 + 9]);
      gyro_raw[i] = static_cast<float>(gyro_raw_u16[i]) / 16.384f * DEG2RAD;
    }

    accl_data_ = rotation_ * Eigen::Matrix<float, 3, 1>(
                                 accl_raw[0], accl_raw[1], accl_raw[2]);

    gyro_data_ = rotation_ * Eigen::Matrix<float, 3, 1>(
                                 gyro_raw[0], gyro_raw[1], gyro_raw[2]) -
                 gyro_data_key_.data_;
  }

  void OnMonitor(void) override {
    if (std::isinf(gyro_data_.x()) || std::isinf(gyro_data_.y()) ||
        std::isinf(gyro_data_.z()) || std::isinf(accl_data_.x()) ||
        std::isinf(accl_data_.y()) || std::isinf(accl_data_.z()) ||
        std::isnan(gyro_data_.x()) || std::isnan(gyro_data_.y()) ||
        std::isnan(gyro_data_.z()) || std::isnan(accl_data_.x()) ||
        std::isnan(accl_data_.y()) || std::isnan(accl_data_.z())) {
      LibXR::STDIO::Printf(
          "ICM42688: NaN data detected. gyro: %f %f %f, accl: %f %f %f\r\n",
          gyro_data_.x(), gyro_data_.y(), gyro_data_.z(), accl_data_.x(),
          accl_data_.y(), accl_data_.z());
    }

    /* Use other timer as HAL timebase (Because the priority of SysTick is
      lowest) and set the priority to the highest to avoid this issue */
    if (fabs(dt_) < 500 || fabs(dt_) > 1500) {
      LibXR::STDIO::Printf("ICM42688 Frequency Error: %6f\r\n",
                           dt_.to_secondf());
    }
  }

  static int CommandFunc(ICM42688<HardwareContainer> *self, int argc,
                         char **argv) {
    if (argc == 1) {
      LibXR::STDIO::Printf("Usage:\r\n");
      LibXR::STDIO::Printf(
          "  show [time_ms] [interval_ms] - Print sensor data "
          "periodically.\r\n");
      LibXR::STDIO::Printf(
          "  list_offset                  - Show current gyro calibration "
          "offset.\r\n");
      LibXR::STDIO::Printf(
          "  cali                         - Start gyroscope calibration.\r\n");
    } else if (argc == 2) {
      if (strcmp(argv[1], "list_offset") == 0) {
        LibXR::STDIO::Printf(
            "Current calibration offset - x: %f, y: %f, z: %f\r\n",
            self->gyro_data_key_.data_.x(), self->gyro_data_key_.data_.y(),
            self->gyro_data_key_.data_.z());
      } else if (strcmp(argv[1], "cali") == 0) {
        self->gyro_data_key_.data_.x() = 0.0,
        self->gyro_data_key_.data_.y() = 0.0,
        self->gyro_data_key_.data_.z() = 0.0;
        LibXR::STDIO::Printf(
            "Starting gyroscope calibration. Please keep the device "
            "steady.\r\n");
        double x = 0.0, y = 0.0, z = 0.0;
        for (int i = 0; i < 30000; i++) {
          x += static_cast<double>(self->gyro_data_.x()) / 30000.0;
          y += static_cast<double>(self->gyro_data_.y()) / 30000.0;
          z += static_cast<double>(self->gyro_data_.z()) / 30000.0;
          if (i % 1000 == 0) {
            LibXR::STDIO::Printf("Progress: %d / 30\r", i / 1000);
          }
          LibXR::Thread::Sleep(1);
        }
        LibXR::STDIO::Printf("\r\nProgress: Done\r\n");

        self->gyro_data_key_.data_.x() = static_cast<float>(x);
        self->gyro_data_key_.data_.y() = static_cast<float>(y);
        self->gyro_data_key_.data_.z() = static_cast<float>(z);

        LibXR::STDIO::Printf("\r\nCalibration result - x: %f, y: %f, z: %f\r\n",
                             self->gyro_data_key_.data_.x(),
                             self->gyro_data_key_.data_.y(),
                             self->gyro_data_key_.data_.z());

        LibXR::STDIO::Printf("Analyzing calibration quality...\r\n");
        x = y = z = 0.0;
        for (int i = 0; i < 30000; i++) {
          x += static_cast<double>(self->gyro_data_.x()) / 30000.0;
          y += static_cast<double>(self->gyro_data_.y()) / 30000.0;
          z += static_cast<double>(self->gyro_data_.z()) / 30000.0;
          if (i % 1000 == 0) {
            LibXR::STDIO::Printf("Progress: %d / 30\r", i / 1000);
          }
          LibXR::Thread::Sleep(1);
        }
        LibXR::STDIO::Printf("\r\nProgress: Done\r\n");

        LibXR::STDIO::Printf("\r\nCalibration error - x: %f, y: %f, z: %f\r\n",
                             x, y, z);

        self->gyro_data_key_.Set(self->gyro_data_key_.data_);
        LibXR::STDIO::Printf("Calibration data saved.\r\n");
      }
    } else if (argc == 4) {
      if (strcmp(argv[1], "show") == 0) {
        int time = std::stoi(argv[2]);
        int delay = std::stoi(argv[3]);

        delay = std::clamp(delay, 2, 1000);

        while (time > 0) {
          LibXR::STDIO::Printf(
              "Accel: x = %+5f, y = %+5f, z = %+5f | "
              "Gyro: x = %+5f, y = %+5f, z = %+5f | Temp: %+5f\r\n",
              self->accl_data_.x(), self->accl_data_.y(), self->accl_data_.z(),
              self->gyro_data_.x(), self->gyro_data_.y(), self->gyro_data_.z(),
              self->temperature_);
          LibXR::Thread::Sleep(delay);
          time -= delay;
        }
      }
    } else {
      LibXR::STDIO::Printf("Error: Invalid arguments.\r\n");
      return -1;
    }
    return 0;
  }

 private:
  float temperature_ = 0.0f;
  float target_temperature_ = 25.0f;

  LibXR::TimestampUS last_int_time_ = 0;
  LibXR::TimestampUS::TimeDiffUS dt_ = 0;

  uint8_t buffer_[ICM42688_READ_LEN];
  Eigen::Matrix<float, 3, 1> gyro_data_, accl_data_;

  LibXR::Topic topic_gyro_, topic_accl_;
  LibXR::GPIO *cs_, *int_;
  LibXR::SPI *spi_;
  LibXR::PWM *pwm_;
  LibXR::Quaternion<float> rotation_;
  LibXR::PID<float> pid_heat_;
  LibXR::Semaphore sem_spi_, new_data_;
  LibXR::SPI::OperationRW op_spi_;
  LibXR::RamFS::File cmd_file_;
  LibXR::Database::Key<Eigen::Matrix<float, 3, 1>> gyro_data_key_;
  LibXR::Thread thread_;
};
