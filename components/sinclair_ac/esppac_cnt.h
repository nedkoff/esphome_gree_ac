// based on: https://github.com/DomiStyle/esphome-panasonic-ac
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "esphome/components/climate/climate.h"
#include "esphome/components/climate/climate_mode.h"
#include "esppac.h"

namespace esphome {
namespace sinclair_ac {
namespace CNT {

enum class ACState {
  Initializing, /* no data for quite a long time */
  Ready,        /* AC talking to us */
};

enum class ACUpdate {
  NoUpdate,    /* no parameters changed - normally process data, static flag set */
  UpdateStart, /* start update with 0xAF and cleared static flag */
  UpdateClear, /* update without 0xAF and cleared static flag */
};

namespace protocol {
  /* SYNC */
  static const uint8_t SYNC = 0x7E;

  /* packet types */
  static const uint8_t CMD_IN_UNIT_REPORT = 0x31;
  static const uint8_t CMD_OUT_PARAMS_SET = 0x01;
  static const uint8_t CMD_OUT_SYNC_TIME  = 0x03;
  static const uint8_t CMD_OUT_MAC_REPORT = 0x04;
  static const uint8_t CMD_OUT_UNKNOWN_1  = 0x02;
  static const uint8_t CMD_IN_UNKNOWN_1   = 0x44;
  static const uint8_t CMD_IN_UNKNOWN_2   = 0x33;

  /* byte indexes are AFTER we remove first 4 bytes from the packet (sync, length, type)
     as well as a checksum */
  static const uint8_t REPORT_PWR_BYTE = 4;
  static const uint8_t REPORT_PWR_MASK = 0b10000000;

  static const uint8_t REPORT_MODE_BYTE = 4;
  static const uint8_t REPORT_MODE_MASK = 0b01110000;
  static const uint8_t REPORT_MODE_POS  = 4;
  static const uint8_t REPORT_MODE_AUTO = 0;
  static const uint8_t REPORT_MODE_COOL = 1;
  static const uint8_t REPORT_MODE_DRY  = 2;
  static const uint8_t REPORT_MODE_FAN  = 3;
  static const uint8_t REPORT_MODE_HEAT = 4;

  static const uint8_t REPORT_FAN_SPD1_BYTE = 18;
  static const uint8_t REPORT_FAN_SPD1_MASK = 0b00001111;
  static const uint8_t REPORT_FAN_SPD1_POS  = 0;

  static const uint8_t REPORT_FAN_SPD2_BYTE = 4;
  static const uint8_t REPORT_FAN_SPD2_MASK = 0b00000011;
  static const uint8_t REPORT_FAN_SPD2_POS  = 0;

  static const uint8_t REPORT_FAN_TURBO_BYTE = 6;
  static const uint8_t REPORT_FAN_TURBO_MASK = 0b00000001;

  static const uint8_t REPORT_TEMP_SET_BYTE = 5;
  static const uint8_t REPORT_TEMP_SET_MASK = 0b11110000;
  static const uint8_t REPORT_TEMP_SET_POS  = 4;
  static const uint8_t REPORT_TEMP_SET_OFF  = 16;

  static const uint8_t REPORT_TEMP_ACT_BYTE = 42;
  static const uint8_t REPORT_TEMP_ACT_MASK = 0b11111111;
  static const uint8_t REPORT_TEMP_ACT_POS  = 0;
  static const uint8_t REPORT_TEMP_ACT_OFF  = 19;
  static const float   REPORT_TEMP_ACT_DIV  = 2.0;

  static const uint8_t REPORT_VSWING_BYTE = 8;
  static const uint8_t REPORT_VSWING_MASK = 0b11110000;
  static const uint8_t REPORT_VSWING_POS  = 4;
  static const uint8_t REPORT_VSWING_OFF  = 0;
  static const uint8_t REPORT_VSWING_FULL  = 1;
  
  static const uint8_t REPORT_DISP_ON_BYTE = 6;
  static const uint8_t REPORT_DISP_ON_MASK = 0b00000010;

  static const uint8_t REPORT_DISP_MODE_BYTE = 9;
  static const uint8_t REPORT_DISP_MODE_MASK = 0b00110000;
  static const uint8_t REPORT_DISP_MODE_POS  = 4;
  static const uint8_t REPORT_DISP_MODE_AUTO = 0;
  static const uint8_t REPORT_DISP_MODE_SET  = 1;
  static const uint8_t REPORT_DISP_MODE_ACT  = 2;
  static const uint8_t REPORT_DISP_MODE_OUT  = 3;

  static const uint8_t REPORT_DISP_F_BYTE = 7;
  static const uint8_t REPORT_DISP_F_MASK = 0b10000000;

  static const uint8_t REPORT_PLASMA1_BYTE = 6;
  static const uint8_t REPORT_PLASMA1_MASK = 0b00000100;
  static const uint8_t REPORT_PLASMA2_BYTE = 0;
  static const uint8_t REPORT_PLASMA2_MASK = 0b00000100;

  static const uint8_t REPORT_SLEEP_BYTE = 4;
  static const uint8_t REPORT_SLEEP_MASK = 0b00001000;

  static const uint8_t REPORT_XFAN_BYTE = 6;
  static const uint8_t REPORT_XFAN_MASK = 0b00001000;

  static const uint8_t REPORT_SAVE_BYTE = 11;
  static const uint8_t REPORT_SAVE_MASK = 0b01000000;

  /* SET packet shares all the byte definition with REPORT */
  static const uint8_t SET_PACKET_LEN = 45;

  static const uint8_t SET_CONST_02_BYTE = 39;
  static const uint8_t SET_CONST_02_VAL  = 0x02;

  static const uint8_t SET_AF_BYTE = 3;
  static const uint8_t SET_AF_VAL  = 0xAF;

  static const uint8_t SET_NOCHANGE_BYTE = 11;
  static const uint8_t SET_NOCHANGE_MASK = 0b00001000;

  static const uint8_t SET_CONST_BIT_BYTE = 7;
  static const uint8_t SET_CONST_BIT_MASK = 0b00000010;

  /* time constraints */
  static const unsigned long TIME_REFRESH_PERIOD_MS   = 300;
  static const unsigned long TIME_TIMEOUT_INACTIVE_MS = 1000;
}  // namespace protocol

/* Define packets from AC that would be processed by software */
const std::vector<uint8_t> allowedPackets = {protocol::CMD_IN_UNIT_REPORT};

class SinclairACCNT : public SinclairAC {
 public:
  void control(const climate::ClimateCall &call) override;

  void on_vertical_swing_change(const std::string &swing) override;

  void on_display_change(const std::string &display) override;
  void on_display_unit_change(const std::string &display_unit) override;

  void on_plasma_change(bool plasma) override;
  void on_sleep_change(bool sleep) override;
  void on_xfan_change(bool xfan) override;
  void on_save_change(bool save) override;

  void setup() override;
  void loop() override;

 protected:
  ACState state_ = ACState::Initializing;
  ACUpdate update_ = ACUpdate::NoUpdate;

  climate::ClimateMode mode_internal_{climate::CLIMATE_MODE_OFF};
  bool power_internal_{false};

  std::string display_mode_internal_;
  bool display_power_internal_{true};

  // --- ACK/Retry (generic) ---
  static constexpr uint32_t PENDING_ACK_TIMEOUT_MS = 800;
  static constexpr uint8_t  PENDING_MAX_RETRIES    = 2;

  enum PendingField : uint16_t {
    PEND_NONE      = 0,
    PEND_MODE_PWR  = 1 << 0,
    PEND_TEMP      = 1 << 1,
    PEND_FAN       = 1 << 2,
    PEND_VSWING    = 1 << 3,
    PEND_DISPLAY   = 1 << 4,
    PEND_DISP_UNIT = 1 << 5,
    PEND_PLASMA    = 1 << 6,
    PEND_SLEEP     = 1 << 7,
    PEND_XFAN      = 1 << 8,
    PEND_SAVE      = 1 << 9,
  };

  uint16_t pending_mask_{0};
  uint32_t pending_deadline_{0};
  uint8_t  pending_retries_{0};

  // desired values (protocol-level)
  bool    pend_power_{false};
  uint8_t pend_mode_{0};        // protocol::REPORT_MODE_*
  uint8_t pend_temp_raw_{0};    // masked (REPORT_TEMP_SET_MASK)

  uint8_t pend_fan1_{0};
  uint8_t pend_fan2_{0};
  bool    pend_turbo_{false};

  uint8_t pend_vswing_{0};      // protocol::REPORT_VSWING_*

  uint8_t pend_disp_mode_{0};   // protocol::REPORT_DISP_MODE_*
  bool    pend_disp_on_{true};

  bool    pend_disp_f_{false};  // Fahrenheit bit

  bool    pend_plasma_{false};
  bool    pend_sleep_{false};
  bool    pend_xfan_{false};
  bool    pend_save_{false};

  void arm_pending_(uint16_t mask);
  void clear_pending_(uint16_t mask);
  bool match_pending_(const std::vector<uint8_t> &report_payload);

  bool processUnitReport();

  void send_packet();

  bool verify_packet();
  void handle_packet();

  climate::ClimateMode determine_mode();
  const char *determine_fan_mode();

  std::string determine_vertical_swing();

  std::string determine_display();
  std::string determine_display_unit();

  bool determine_plasma();
  bool determine_sleep();
  bool determine_xfan();
  bool determine_save();
};

}  // namespace CNT
}  // namespace sinclair_ac
}  // namespace esphome
