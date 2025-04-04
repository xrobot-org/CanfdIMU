#pragma once

// clang-format off
/* === MODULE MANIFEST ===
module_name: CanfdIMU
module_description: CANFD/串口IMU通信模块 CANFD/UART IMU Communication Module
constructor_args:
  - accl_topic: "imu_accl"
  - gyro_topic: "imu_gyro"
  - quat_topic: "imu_quat"
  - eulr_topic: "imu_eulr"
  - task_stack_depth_uart: 384
  - task_stack_depth_can: 384
required_hardware: imu_fdcan imu_data_uart ramfs database
repository: https://github.com/xrobot-org/CanfdIMU
=== END MANIFEST === */
// clang-format on

#include <cstring>

#include "app_framework.hpp"
#include "can.hpp"
#include "crc.hpp"
#include "database.hpp"
#include "message.hpp"
#include "timebase.hpp"
#include "uart.hpp"

template <typename HardwareContainer>
class CanfdIMU : public LibXR::Application {
 public:
  explicit CanfdIMU(HardwareContainer& hw, LibXR::ApplicationManager& app,
                    const char* accl_topic, const char* gyro_topic,
                    const char* quat_topic, const char* eulr_topic,
                    uint32_t task_stack_depth_uart,
                    uint32_t task_stack_depth_can)
      : accl_topic_name_(accl_topic),
        gyro_topic_name_(gyro_topic),
        quat_topic_name_(quat_topic),
        eulr_topic_name_(eulr_topic),
        can_(hw.template FindOrExit<LibXR::FDCAN>({"imu_fdcan"})),
        uart_(hw.template FindOrExit<LibXR::UART>({"imu_data_uart"})),
        config_(*hw.template FindOrExit<LibXR::Database>({"database"}),
                "canfd_imu",
                Configuration{0x30, 1, 1000000, LibXR::UART::Parity::NO_PARITY,
                              false, true, true, true, false, false, true}),
        cmd_file_(LibXR::RamFS::CreateFile("set_imu", CommandFunc, this)) {
    app.Register(*this);

    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    thread_uart_.Create(this, ThreadUart, "canfd_imu_uart",
                        task_stack_depth_uart, LibXR::Thread::Priority::MEDIUM);
    thread_can_.Create(this, ThreadCan, "canfd_imu_can", task_stack_depth_can,
                       LibXR::Thread::Priority::MEDIUM);
  }

  void OnMonitor() override {
    // Optional: Add self-check, debug output, frequency monitor, etc.
  }

  static int CommandFunc(CanfdIMU<HardwareContainer>* imu, int argc,
                         char** argv) {
    if (argc == 1) {
      if (imu->config_.data_.canfd_enabled) {
        LibXR::STDIO::Printf("canfd mode\r\n");
      } else if (imu->config_.data_.can_enabled) {
        LibXR::STDIO::Printf("can mode\r\ndata:");
        if (imu->config_.data_.accl_enabled) {
          LibXR::STDIO::Printf("accl,");
        }
        if (imu->config_.data_.gyro_enabled) {
          LibXR::STDIO::Printf("gyro,");
        }
        if (imu->config_.data_.quat_enabled) {
          LibXR::STDIO::Printf("quat,");
        }
        if (imu->config_.data_.eulr_enabled) {
          LibXR::STDIO::Printf("eulr,");
        }
        LibXR::STDIO::Printf("\r\n");
      } else {
        LibXR::STDIO::Printf("can/canfd output disabled.\r\n");
      }

      if (imu->config_.data_.uart_enabled) {
        LibXR::STDIO::Printf("uart output enabled.\r\n");
      } else {
        LibXR::STDIO::Printf("uart output disabled.\r\n");
      }

      LibXR::STDIO::Printf("feedback delay:%d\r\n",
                           imu->config_.data_.fb_cycle);
      LibXR::STDIO::Printf("id:%d\r\n\r\nUsage:\r\n", imu->config_.data_.id);
      LibXR::STDIO::Printf("\tset_delay  [time]  设置发送延时ms\r\n");
      LibXR::STDIO::Printf("\tset_can_id [id]    设置can id\r\n");
      LibXR::STDIO::Printf(
          "\tenable/disable     "
          "[accl/gyro/quat/eulr/canfd/can/uart/can_bridge]\r\n");
      LibXR::STDIO::Printf(
          "\tset_uart_baud      [0:9600 1:100000 2:115200 3:460800 4:921600 "
          "5:1000000 6:2000000]\r\n");
      LibXR::STDIO::Printf("\tset_uart_parity    [0:None, 1:Odd, 2:Even]\r\n");
    } else if (argc == 3 && strcmp(argv[1], "set_delay") == 0) {
      int delay = std::stoi(argv[2]);

      if (delay > 1000) {
        delay = 1000;
      }

      if (delay < 1) {
        delay = 1;
      }

      imu->config_.data_.fb_cycle = delay;

      LibXR::STDIO::Printf("delay:%d\r\n", delay);

      imu->config_.Set(imu->config_.data_);
    } else if (argc == 3 && strcmp(argv[1], "enable") == 0) {
      if (strcmp(argv[2], "accl") == 0) {
        imu->config_.data_.accl_enabled = true;
      } else if (strcmp(argv[2], "gyro") == 0) {
        imu->config_.data_.gyro_enabled = true;
      } else if (strcmp(argv[2], "quat") == 0) {
        imu->config_.data_.quat_enabled = true;
      } else if (strcmp(argv[2], "eulr") == 0) {
        imu->config_.data_.eulr_enabled = true;
      } else if (strcmp(argv[2], "canfd") == 0) {
        imu->config_.data_.canfd_enabled = true;
        if (!imu->config_.data_.can_enabled) {
          imu->config_.data_.can_enabled = true;
        }
      } else if (strcmp(argv[2], "can") == 0) {
        imu->config_.data_.can_enabled = true;
        if (imu->config_.data_.canfd_enabled) {
          imu->config_.data_.canfd_enabled = false;
        }
      } else if (strcmp(argv[2], "uart") == 0) {
        imu->config_.data_.uart_enabled = true;
      } else {
        LibXR::STDIO::Printf("命令错误\r\n");
        return -1;
      }

      imu->config_.Set(imu->config_.data_);
    } else if (argc == 3 && strcmp(argv[1], "disable") == 0) {
      if (strcmp(argv[2], "accl") == 0) {
        imu->config_.data_.accl_enabled = false;
      } else if (strcmp(argv[2], "gyro") == 0) {
        imu->config_.data_.gyro_enabled = false;
      } else if (strcmp(argv[2], "quat") == 0) {
        imu->config_.data_.quat_enabled = false;
      } else if (strcmp(argv[2], "eulr") == 0) {
        imu->config_.data_.eulr_enabled = false;
      } else if (strcmp(argv[2], "canfd") == 0) {
        imu->config_.data_.canfd_enabled = false;
      } else if (strcmp(argv[2], "can") == 0) {
        imu->config_.data_.can_enabled = false;
      } else if (strcmp(argv[2], "uart") == 0) {
        imu->config_.data_.uart_enabled = false;
      } else {
        LibXR::STDIO::Printf("命令错误\r\n");
        return -1;
      }

      imu->config_.Set(imu->config_.data_);
    } else if (argc == 3 && strcmp(argv[1], "set_can_id") == 0) {
      int id = std::stoi(argv[2]);

      imu->config_.data_.id = id;

      LibXR::STDIO::Printf("can_id:%d\r\n", id);

      imu->config_.Set(imu->config_.data_);
    } else if (argc == 3 && strcmp(argv[1], "set_uart_baud") == 0) {
      uint32_t baud = std::stoi(argv[2]);
      if (baud > 6) {
        LibXR::STDIO::Printf("baud should be between 0 and 6\r\n");
        return -1;
      }

      static const uint32_t UART_BAUD_MAP[] = {9600,   100000,  115200, 460800,
                                               921600, 1000000, 2000000};

      imu->config_.data_.uart_baudrate = UART_BAUD_MAP[baud];
      LibXR::STDIO::Printf("baud:%d\r\n", UART_BAUD_MAP[baud]);
      imu->config_.Set(imu->config_.data_);
    } else if (argc == 3 && strcmp(argv[1], "set_uart_parity") == 0) {
      uint32_t parity = std::stoi(argv[2]);
      if (parity > 2) {
        LibXR::STDIO::Printf("parity should be between 0 and 2\r\n");
        return -1;
      }

      imu->config_.data_.uart_parity = static_cast<LibXR::UART::Parity>(parity);
      imu->config_.Set(imu->config_.data_);
    } else {
      LibXR::STDIO::Printf("命令错误\r\n");
    }

    return 0;
  }

 private:
  const char* accl_topic_name_;
  const char* gyro_topic_name_;
  const char* quat_topic_name_;
  const char* eulr_topic_name_;

  struct __attribute__((packed)) Configuration {
    uint32_t id;
    uint32_t fb_cycle;
    uint32_t uart_baudrate;
    LibXR::UART::Parity uart_parity;
    bool canfd_enabled;
    bool can_enabled;
    bool uart_enabled;
    bool eulr_enabled;
    bool quat_enabled;
    bool accl_enabled;
    bool gyro_enabled;
  };

  struct __attribute__((packed)) Data {
    uint8_t prefix = 0xA5;
    uint8_t id = 0x30;  // Default ID
    uint32_t time = 0;
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f};
    float accl[3] = {0.0f};
    float eulr[3] = {0.0f};
    uint8_t crc8 = 0;
  };

  struct __attribute__((packed)) DataCanfd {
    uint32_t time = 0;
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro[3] = {0.0f};
    float accl[3] = {0.0f};
    float eulr[3] = {0.0f};
  };

  struct __attribute__((packed)) CanData3 {
    struct __attribute__((packed)) {
      int32_t data1 : 21;
      int32_t data2 : 21;
      int32_t data3 : 21;
      int32_t res : 1;
    };

    struct __attribute__((packed)) {
      uint32_t data1_unsigned : 21;
      uint32_t data2_unsigned : 21;
      uint32_t data3_unsigned : 21;
      uint32_t res_unsigned : 1;
    };
  };

  struct __attribute__((packed)) CanData4 {
    union {
      int16_t data[4];
      uint16_t data_unsigned[4];
    };
  };

  enum class CanPackID : uint32_t { ACCL = 0, GYRO = 1, EULR = 3, QUAT = 4 };

  LibXR::FDCAN* can_;
  LibXR::UART* uart_;
  LibXR::Database::Key<Configuration> config_;

  LibXR::RamFS::File cmd_file_;

  LibXR::Thread thread_uart_;
  LibXR::Thread thread_can_;
  LibXR::Semaphore uart_write_sem_;

  LibXR::Quaternion<float> quat_ = {1.0f, 0.0f, 0.0f, 0.0f};
  LibXR::EulerAngle<float> eulr_ = {0.0f, 0.0f, 0.0f};
  Eigen::Matrix<float, 3, 1> gyro_ = {0.0f, 0.0f, 0.0f};
  Eigen::Matrix<float, 3, 1> accl_ = {0.0f, 0.0f, 0.0f};

  static void ThreadUart(CanfdIMU* self) {
    self->uart_->SetConfig({self->config_.data_.uart_baudrate,
                            self->config_.data_.uart_parity, 8, 1});

    auto sub_accl = LibXR::Topic(self->accl_topic_name_, sizeof(self->accl_));
    auto sub_gyro = LibXR::Topic(self->gyro_topic_name_, sizeof(self->gyro_));
    auto sub_quat = LibXR::Topic(self->quat_topic_name_, sizeof(self->quat_));
    auto sub_eulr = LibXR::Topic(self->eulr_topic_name_, sizeof(self->eulr_));

    Data send_buffer = {};
    LibXR::WriteOperation write_op(self->uart_write_sem_);

    auto last_waskup_time = LibXR::Timebase::GetMilliseconds();

    while (true) {
      sub_accl.DumpData(self->accl_);
      sub_gyro.DumpData(self->gyro_);
      sub_quat.DumpData(self->quat_);
      sub_eulr.DumpData(self->eulr_);

      if (self->config_.data_.uart_enabled) {
        send_buffer.prefix = 0xA5;
        send_buffer.id = self->config_.data_.id;
        send_buffer.time = LibXR::Timebase::GetMilliseconds();
        send_buffer.quat[0] = self->quat_.w();
        send_buffer.quat[1] = self->quat_.x();
        send_buffer.quat[2] = self->quat_.y();
        send_buffer.quat[3] = self->quat_.z();
        memcpy(send_buffer.gyro, self->gyro_.data(), sizeof(send_buffer.gyro));
        memcpy(send_buffer.accl, self->accl_.data(), sizeof(send_buffer.accl));
        memcpy(send_buffer.eulr, self->eulr_.data_, sizeof(send_buffer.eulr));
        send_buffer.crc8 = LibXR::CRC8::Calculate(
            reinterpret_cast<const uint8_t*>(&send_buffer),
            sizeof(Data) - sizeof(uint8_t));

        self->uart_->write_port_(send_buffer, write_op);
      }

      LibXR::Thread::SleepUntil(last_waskup_time, self->config_.data_.fb_cycle);
    }
  }

  static void ThreadCan(CanfdIMU* self) {
    auto last_waskup_time = LibXR::Timebase::GetMilliseconds();

    LibXR::FDCAN::FDPack pack = {};

    LibXR::CAN::ClassicPack classic_pack = {};

    CanData3* can_data3 = reinterpret_cast<CanData3*>(classic_pack.data);
    CanData4* can_data4 = reinterpret_cast<CanData4*>(classic_pack.data);

    DataCanfd send_buffer = {};

    while (true) {
      if (self->config_.data_.canfd_enabled) {
        send_buffer.time = LibXR::Timebase::GetMilliseconds();
        send_buffer.quat[0] = self->quat_.w();
        send_buffer.quat[1] = self->quat_.x();
        send_buffer.quat[2] = self->quat_.y();
        send_buffer.quat[3] = self->quat_.z();
        memcpy(send_buffer.gyro, self->gyro_.data(), sizeof(send_buffer.gyro));
        memcpy(send_buffer.accl, self->accl_.data(), sizeof(send_buffer.accl));
        memcpy(send_buffer.eulr, self->eulr_.data_, sizeof(send_buffer.eulr));

        memcpy(pack.data, &send_buffer, sizeof(send_buffer));
        pack.type = LibXR::CAN::Type::STANDARD;
        pack.id = self->config_.data_.id;
        pack.len = sizeof(send_buffer);

        self->can_->AddMessage(pack);
      } else if (self->config_.data_.can_enabled) {
        classic_pack.type = LibXR::CAN::Type::STANDARD;
        if (self->config_.data_.gyro_enabled) {
          classic_pack.id =
              self->config_.data_.id + static_cast<uint32_t>(CanPackID::GYRO);
          constexpr float ENCODE_FACTOR =
              1.0 / (M_PI / 180.0 / 2000) * (static_cast<double>(1 << 20) - 1);
          can_data3->data1 = self->gyro_.x() * ENCODE_FACTOR;
          can_data3->data2 = self->gyro_.y() * ENCODE_FACTOR;
          can_data3->data3 = self->gyro_.z() * ENCODE_FACTOR;
          self->can_->AddMessage(classic_pack);
        }

        if (self->config_.data_.accl_enabled) {
          classic_pack.id =
              self->config_.data_.id + static_cast<uint32_t>(CanPackID::ACCL);
          constexpr float ENCODE_FACTOR =
              1.0 / 24.0 * (static_cast<double>(1 << 20) - 1);
          can_data3->data1 = self->accl_.x() * ENCODE_FACTOR;
          can_data3->data2 = self->accl_.y() * ENCODE_FACTOR;
          can_data3->data3 = self->accl_.z() * ENCODE_FACTOR;
          self->can_->AddMessage(classic_pack);
        }

        if (self->config_.data_.eulr_enabled) {
          classic_pack.id =
              self->config_.data_.id + static_cast<uint32_t>(CanPackID::EULR);
          constexpr float ENCODE_FACTOR =
              1.0 / M_2PI * (static_cast<double>(1 << 20) - 1);
          can_data3->data1 = self->eulr_.pitch_ * ENCODE_FACTOR;
          can_data3->data2 = self->eulr_.roll_ * ENCODE_FACTOR;
          can_data3->data3 = self->eulr_.yaw_ * ENCODE_FACTOR;
          self->can_->AddMessage(classic_pack);
        }

        if (self->config_.data_.quat_enabled) {
          classic_pack.id =
              self->config_.data_.id + static_cast<uint32_t>(CanPackID::QUAT);
          constexpr float ENCODE_FACTOR = 1.0 * INT16_MAX;
          can_data4->data[0] = self->quat_.w() * ENCODE_FACTOR;
          can_data4->data[1] = self->quat_.x() * ENCODE_FACTOR;
          can_data4->data[2] = self->quat_.y() * ENCODE_FACTOR;
          can_data4->data[3] = self->quat_.z() * ENCODE_FACTOR;
          self->can_->AddMessage(classic_pack);
        }
      }

      LibXR::Thread::SleepUntil(last_waskup_time, self->config_.data_.fb_cycle);
    }
  }
};
