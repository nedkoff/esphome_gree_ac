// based on: https://github.com/DomiStyle/esphome-panasonic-ac
#include "esppac_cnt.h"

#include <cstring>

namespace esphome {
namespace sinclair_ac {
namespace CNT {

static const char *const TAG = "sinclair_ac.serial";

// -------------------- pending helpers --------------------

void SinclairACCNT::arm_pending_(uint16_t mask) {
  this->pending_mask_ |= mask;
  this->pending_retries_ = 0;
  this->pending_deadline_ = millis() + PENDING_ACK_TIMEOUT_MS;
}

void SinclairACCNT::clear_pending_(uint16_t mask) {
  this->pending_mask_ &= ~mask;
  if (this->pending_mask_ == 0) {
    this->pending_retries_ = 0;
  }
}

bool SinclairACCNT::match_pending_(const std::vector<uint8_t> &r) {
  // r = payload after stripping (7E 7E LEN CMD) and checksum
  if (r.size() < protocol::SET_PACKET_LEN) {
    // report payload should be 45 bytes; if not, don't touch anything
    return false;
  }

  // MODE + POWER
  if (this->pending_mask_ & PEND_MODE_PWR) {
    bool pwr = (r[protocol::REPORT_PWR_BYTE] & protocol::REPORT_PWR_MASK) != 0;
    uint8_t mode = (r[protocol::REPORT_MODE_BYTE] & protocol::REPORT_MODE_MASK) >> protocol::REPORT_MODE_POS;
    if (pwr == this->pend_power_ && mode == this->pend_mode_) {
      this->clear_pending_(PEND_MODE_PWR);
    }
  }

  // TEMP (set)
  if (this->pending_mask_ & PEND_TEMP) {
    uint8_t raw = (r[protocol::REPORT_TEMP_SET_BYTE] & protocol::REPORT_TEMP_SET_MASK);
    if (raw == this->pend_temp_raw_) {
      this->clear_pending_(PEND_TEMP);
    }
  }

  // FAN
  // FAN
if (this->pending_mask_ & PEND_FAN) {
  // NOTE: on some Sinclair variants SPD1 is not reliably reported back,
  // so we ACK fan changes using SPD2 + TURBO only.
  uint8_t f2 = (r[protocol::REPORT_FAN_SPD2_BYTE] & protocol::REPORT_FAN_SPD2_MASK) >> protocol::REPORT_FAN_SPD2_POS;
  bool turbo = (r[protocol::REPORT_FAN_TURBO_BYTE] & protocol::REPORT_FAN_TURBO_MASK) != 0;

  if (f2 == this->pend_fan2_ && turbo == this->pend_turbo_) {
    this->clear_pending_(PEND_FAN);
  }
}


  // VSWING
  if (this->pending_mask_ & PEND_VSWING) {
    uint8_t vs = (r[protocol::REPORT_VSWING_BYTE] & protocol::REPORT_VSWING_MASK) >> protocol::REPORT_VSWING_POS;
    if (vs == this->pend_vswing_) {
      this->clear_pending_(PEND_VSWING);
    }
  }

  // DISPLAY
  if (this->pending_mask_ & PEND_DISPLAY) {
    uint8_t dm = (r[protocol::REPORT_DISP_MODE_BYTE] & protocol::REPORT_DISP_MODE_MASK) >> protocol::REPORT_DISP_MODE_POS;
    bool on = (r[protocol::REPORT_DISP_ON_BYTE] & protocol::REPORT_DISP_ON_MASK) != 0;
    if (dm == this->pend_disp_mode_ && on == this->pend_disp_on_) {
      this->clear_pending_(PEND_DISPLAY);
    }
  }

  // DISPLAY UNIT
  if (this->pending_mask_ & PEND_DISP_UNIT) {
    bool f = (r[protocol::REPORT_DISP_F_BYTE] & protocol::REPORT_DISP_F_MASK) != 0;
    if (f == this->pend_disp_f_) {
      this->clear_pending_(PEND_DISP_UNIT);
    }
  }

  // PLASMA
  if (this->pending_mask_ & PEND_PLASMA) {
    bool p1 = (r[protocol::REPORT_PLASMA1_BYTE] & protocol::REPORT_PLASMA1_MASK) != 0;
    bool p2 = (r[protocol::REPORT_PLASMA2_BYTE] & protocol::REPORT_PLASMA2_MASK) != 0;
    bool pl = (p1 || p2);
    if (pl == this->pend_plasma_) {
      this->clear_pending_(PEND_PLASMA);
    }
  }

  // SLEEP
  if (this->pending_mask_ & PEND_SLEEP) {
    bool s = (r[protocol::REPORT_SLEEP_BYTE] & protocol::REPORT_SLEEP_MASK) != 0;
    if (s == this->pend_sleep_) {
      this->clear_pending_(PEND_SLEEP);
    }
  }

  // XFAN
  if (this->pending_mask_ & PEND_XFAN) {
    bool x = (r[protocol::REPORT_XFAN_BYTE] & protocol::REPORT_XFAN_MASK) != 0;
    if (x == this->pend_xfan_) {
      this->clear_pending_(PEND_XFAN);
    }
  }

  // SAVE
  if (this->pending_mask_ & PEND_SAVE) {
    bool sv = (r[protocol::REPORT_SAVE_BYTE] & protocol::REPORT_SAVE_MASK) != 0;
    if (sv == this->pend_save_) {
      this->clear_pending_(PEND_SAVE);
    }
  }

  return this->pending_mask_ == 0;
}

// -------------------- component --------------------

void SinclairACCNT::setup() {
  SinclairAC::setup();
  ESP_LOGD(TAG, "Using serial protocol for Sinclair AC");
}

void SinclairACCNT::loop() {
  /* this reads data from UART */
  SinclairAC::loop();

  /* we have a frame from AC */
  if (this->serialProcess_.state == STATE_COMPLETE) {
    /* do not forget to order for restart of the recieve state machine */
    this->serialProcess_.state = STATE_RESTART;
    /* mark that we have recieved a response */
    this->wait_response_ = false;
    /* log for ESPHome debug */
    log_packet(this->serialProcess_.data);

    if (!verify_packet()) { /* Verify length, header, counter and checksum */
      return;
    }

    this->last_packet_received_ = millis();

    
    // --- PEEK REPORT for pending ACKs (do not change internal state) ---
    if (this->pending_mask_ && this->serialProcess_.data[3] == protocol::CMD_IN_UNIT_REPORT) {
      auto tmp = this->serialProcess_.data;
      tmp.erase(tmp.begin(), tmp.begin() + 4);  // strip header
      tmp.pop_back();                           // strip checksum

      uint16_t before = this->pending_mask_;
      this->match_pending_(tmp);

      if (before != this->pending_mask_) {
        ESP_LOGD(TAG, "ACK progress: 0x%04X -> 0x%04X", (unsigned) before, (unsigned) this->pending_mask_);
      }
    }

    /* A valid recieved packet of accepted type marks module as being ready */
    if (this->state_ != ACState::Ready) {
      this->state_ = ACState::Ready;
      Component::status_clear_error();
      this->last_packet_sent_ = millis();
    }

    if (this->update_ == ACUpdate::NoUpdate) {
      handle_packet(); /* this will update state of components in HA as well as internal settings */
    }
  }

  /* we will send a packet to the AC as a reponse to indicate changes */
  send_packet();

  // --- RETRY pending if no ACK in time ---
  if (this->pending_mask_ && (int32_t) (millis() - this->pending_deadline_) >= 0) {
    if (this->pending_retries_ < PENDING_MAX_RETRIES) {
      this->pending_retries_++;
      this->pending_deadline_ = millis() + PENDING_ACK_TIMEOUT_MS;

      ESP_LOGW(TAG, "ACK timeout, retry %u, pending_mask=0x%04X",
               (unsigned) this->pending_retries_, (unsigned) this->pending_mask_);

      // allow resend even if we were "waiting"
      this->wait_response_ = false;

      // trigger resend of current desired state
      this->update_ = ACUpdate::UpdateStart;
    } else {
      ESP_LOGW(TAG, "ACK failed after retries, giving up, pending_mask=0x%04X", (unsigned) this->pending_mask_);
      this->pending_mask_ = 0;
    }
  }

  /* if there are no packets for 5 seconds - mark module as not ready */
  if (millis() - this->last_packet_received_ >= protocol::TIME_TIMEOUT_INACTIVE_MS) {
    if (this->state_ != ACState::Initializing) {
      this->state_ = ACState::Initializing;
      Component::status_set_error();
    }
  }
}

/*
 * ESPHome control request
 */

void SinclairACCNT::control(const climate::ClimateCall &call) {
  if (this->state_ != ACState::Ready)
    return;

  if (call.get_mode().has_value()) {
    ESP_LOGV(TAG, "Requested mode change");
    this->update_ = ACUpdate::UpdateStart;
    this->mode = *call.get_mode();

    // arm pending MODE+PWR in protocol terms (must match send_packet logic)
    uint8_t mode = protocol::REPORT_MODE_AUTO;
    bool power = false;
    switch (this->mode) {
      case climate::CLIMATE_MODE_AUTO:     mode = protocol::REPORT_MODE_AUTO; power = true; break;
      case climate::CLIMATE_MODE_COOL:     mode = protocol::REPORT_MODE_COOL; power = true; break;
      case climate::CLIMATE_MODE_DRY:      mode = protocol::REPORT_MODE_DRY;  power = true; break;
      case climate::CLIMATE_MODE_FAN_ONLY: mode = protocol::REPORT_MODE_FAN;  power = true; break;
      case climate::CLIMATE_MODE_HEAT:     mode = protocol::REPORT_MODE_HEAT; power = true; break;
      default:
      case climate::CLIMATE_MODE_OFF:
        power = false;
        switch (this->mode_internal_) {
          case climate::CLIMATE_MODE_AUTO:     mode = protocol::REPORT_MODE_AUTO; break;
          case climate::CLIMATE_MODE_COOL:     mode = protocol::REPORT_MODE_COOL; break;
          case climate::CLIMATE_MODE_DRY:      mode = protocol::REPORT_MODE_DRY;  break;
          case climate::CLIMATE_MODE_FAN_ONLY: mode = protocol::REPORT_MODE_FAN;  break;
          case climate::CLIMATE_MODE_HEAT:     mode = protocol::REPORT_MODE_HEAT; break;
          default:                             mode = protocol::REPORT_MODE_AUTO; break;
        }
        break;
    }

    this->pend_mode_ = mode;
    this->pend_power_ = power;
    this->arm_pending_(PEND_MODE_PWR);
  }

  if (call.get_target_temperature().has_value()) {
    ESP_LOGV(TAG, "Requested target teperature change");
    this->update_ = ACUpdate::UpdateStart;
    this->target_temperature = *call.get_target_temperature();

    if (this->target_temperature < MIN_TEMPERATURE) {
      this->target_temperature = MIN_TEMPERATURE;
    } else if (this->target_temperature > MAX_TEMPERATURE) {
      this->target_temperature = MAX_TEMPERATURE;
    }

    this->pend_temp_raw_ =
        ((((uint8_t) this->target_temperature) - protocol::REPORT_TEMP_SET_OFF) << protocol::REPORT_TEMP_SET_POS) &
        protocol::REPORT_TEMP_SET_MASK;
    this->arm_pending_(PEND_TEMP);
  }

  if (call.has_custom_fan_mode()) {
    ESP_LOGV(TAG, "Requested fan mode change");
    this->update_ = ACUpdate::UpdateStart;
    this->set_custom_fan_mode_(call.get_custom_fan_mode());

    // map to protocol fan values
    uint8_t fanSpeed1 = 0;
    uint8_t fanSpeed2 = 0;
    bool fanTurbo = false;

    if (this->has_custom_fan_mode()) {
      const char *m = this->get_custom_fan_mode();

      if (strcmp(m, fan_modes::FAN_AUTO) == 0)        { fanSpeed1 = 0; fanSpeed2 = 0; fanTurbo = false; }
      else if (strcmp(m, fan_modes::FAN_LOW) == 0)    { fanSpeed1 = 1; fanSpeed2 = 1; fanTurbo = false; }
      else if (strcmp(m, fan_modes::FAN_MED) == 0)    { fanSpeed1 = 3; fanSpeed2 = 2; fanTurbo = false; }
      else if (strcmp(m, fan_modes::FAN_HIGH) == 0)   { fanSpeed1 = 5; fanSpeed2 = 3; fanTurbo = false; }
      else if (strcmp(m, fan_modes::FAN_TURBO) == 0)  { fanSpeed1 = 5; fanSpeed2 = 3; fanTurbo = true; }
      else                                            { fanSpeed1 = 0; fanSpeed2 = 0; fanTurbo = false; }
    }

    this->pend_fan1_ = fanSpeed1;
    this->pend_fan2_ = fanSpeed2;
    this->pend_turbo_ = fanTurbo;
    this->arm_pending_(PEND_FAN);
  }

  if (call.get_swing_mode().has_value()) {
    ESP_LOGV(TAG, "Requested swing mode change");
    this->update_ = ACUpdate::UpdateStart;

    switch (*call.get_swing_mode()) {
      case climate::CLIMATE_SWING_OFF:
        this->vertical_swing_state_ = vertical_swing_options::CMID;
        break;
      case climate::CLIMATE_SWING_VERTICAL:
        this->vertical_swing_state_ = vertical_swing_options::FULL;
        break;
      default:
        this->vertical_swing_state_ = vertical_swing_options::CMID;
        break;
    }

    uint8_t vs = protocol::REPORT_VSWING_OFF;
    if (this->vertical_swing_state_ == vertical_swing_options::FULL) vs = protocol::REPORT_VSWING_FULL;
    else if (this->vertical_swing_state_ == vertical_swing_options::CMID) vs = protocol::REPORT_VSWING_CMID;
    else vs = protocol::REPORT_VSWING_OFF;

    this->pend_vswing_ = vs;
    this->arm_pending_(PEND_VSWING);
  }
}

/*
 * Send a raw packet, as is
 */

void SinclairACCNT::send_packet() {
  std::vector<uint8_t> packet(protocol::SET_PACKET_LEN, 0);

  if (this->wait_response_ == true && (millis() - this->last_packet_sent_) < protocol::TIME_REFRESH_PERIOD_MS) {
    return;
  }

  packet[protocol::SET_CONST_02_BYTE] = protocol::SET_CONST_02_VAL;
  packet[protocol::SET_CONST_BIT_BYTE] = protocol::SET_CONST_BIT_MASK;

  switch (this->update_) {
    default:
    case ACUpdate::NoUpdate:
      packet[protocol::SET_NOCHANGE_BYTE] |= protocol::SET_NOCHANGE_MASK;
      break;
    case ACUpdate::UpdateStart:
      packet[protocol::SET_AF_BYTE] = protocol::SET_AF_VAL;
      break;
    case ACUpdate::UpdateClear:
      break;
  }

  // MODE and POWER
  uint8_t mode = protocol::REPORT_MODE_AUTO;
  bool power = false;
  switch (this->mode) {
    case climate::CLIMATE_MODE_AUTO:     mode = protocol::REPORT_MODE_AUTO; power = true; break;
    case climate::CLIMATE_MODE_COOL:     mode = protocol::REPORT_MODE_COOL; power = true; break;
    case climate::CLIMATE_MODE_DRY:      mode = protocol::REPORT_MODE_DRY;  power = true; break;
    case climate::CLIMATE_MODE_FAN_ONLY: mode = protocol::REPORT_MODE_FAN;  power = true; break;
    case climate::CLIMATE_MODE_HEAT:     mode = protocol::REPORT_MODE_HEAT; power = true; break;
    default:
    case climate::CLIMATE_MODE_OFF:
      switch (this->mode_internal_) {
        case climate::CLIMATE_MODE_AUTO:     mode = protocol::REPORT_MODE_AUTO; break;
        case climate::CLIMATE_MODE_COOL:     mode = protocol::REPORT_MODE_COOL; break;
        case climate::CLIMATE_MODE_DRY:      mode = protocol::REPORT_MODE_DRY;  break;
        case climate::CLIMATE_MODE_FAN_ONLY: mode = protocol::REPORT_MODE_FAN;  break;
        case climate::CLIMATE_MODE_HEAT:     mode = protocol::REPORT_MODE_HEAT; break;
        default:                             mode = protocol::REPORT_MODE_AUTO; break;
      }
      power = false;
      break;
  }

  packet[protocol::REPORT_MODE_BYTE] |= (mode << protocol::REPORT_MODE_POS);
  if (power) packet[protocol::REPORT_PWR_BYTE] |= protocol::REPORT_PWR_MASK;

  // TARGET TEMP
  uint8_t target_temperature =
      ((((uint8_t) this->target_temperature) - protocol::REPORT_TEMP_SET_OFF) << protocol::REPORT_TEMP_SET_POS);
  packet[protocol::REPORT_TEMP_SET_BYTE] |= (target_temperature & protocol::REPORT_TEMP_SET_MASK);

  // FAN
  uint8_t fanSpeed1 = 0, fanSpeed2 = 0;
  bool fanTurbo = false;
  if (this->has_custom_fan_mode()) {
    const char *custom_fan_mode = this->get_custom_fan_mode();
    if (strcmp(custom_fan_mode, fan_modes::FAN_AUTO) == 0)        { fanSpeed1 = 0; fanSpeed2 = 0; fanTurbo = false; }
    else if (strcmp(custom_fan_mode, fan_modes::FAN_LOW) == 0)    { fanSpeed1 = 1; fanSpeed2 = 1; fanTurbo = false; }
    else if (strcmp(custom_fan_mode, fan_modes::FAN_MED) == 0)    { fanSpeed1 = 3; fanSpeed2 = 2; fanTurbo = false; }
    else if (strcmp(custom_fan_mode, fan_modes::FAN_HIGH) == 0)   { fanSpeed1 = 5; fanSpeed2 = 3; fanTurbo = false; }
    else if (strcmp(custom_fan_mode, fan_modes::FAN_TURBO) == 0)  { fanSpeed1 = 5; fanSpeed2 = 3; fanTurbo = true; }
    else                                                          { fanSpeed1 = 0; fanSpeed2 = 0; fanTurbo = false; }
  }

  packet[protocol::REPORT_FAN_SPD1_BYTE] |= (fanSpeed1 << protocol::REPORT_FAN_SPD1_POS);
  packet[protocol::REPORT_FAN_SPD2_BYTE] |= (fanSpeed2 << protocol::REPORT_FAN_SPD2_POS);
  if (fanTurbo) packet[protocol::REPORT_FAN_TURBO_BYTE] |= protocol::REPORT_FAN_TURBO_MASK;

  // VSWING
  uint8_t mode_vertical_swing = protocol::REPORT_VSWING_OFF;
  if (this->vertical_swing_state_ == vertical_swing_options::OFF) { mode_vertical_swing = protocol::REPORT_VSWING_OFF; }
  else if (this->vertical_swing_state_ == vertical_swing_options::FULL) { mode_vertical_swing = protocol::REPORT_VSWING_FULL; }
  else { mode_vertical_swing = protocol::REPORT_VSWING_OFF; }


  packet[protocol::REPORT_VSWING_BYTE] |= (mode_vertical_swing << protocol::REPORT_VSWING_POS);

  // DISPLAY
  uint8_t display_mode = protocol::REPORT_DISP_MODE_AUTO;
  if (this->display_state_ == display_options::AUTO) { display_mode = protocol::REPORT_DISP_MODE_AUTO; this->display_power_internal_ = true; }
  else if (this->display_state_ == display_options::SET) { display_mode = protocol::REPORT_DISP_MODE_SET; this->display_power_internal_ = true; }
  else if (this->display_state_ == display_options::ACT) { display_mode = protocol::REPORT_DISP_MODE_ACT; this->display_power_internal_ = true; }
  else if (this->display_state_ == display_options::OUT) { display_mode = protocol::REPORT_DISP_MODE_OUT; this->display_power_internal_ = true; }
  else if (this->display_state_ == display_options::OFF) {
    this->display_power_internal_ = false;
    if (this->display_mode_internal_ == display_options::SET) display_mode = protocol::REPORT_DISP_MODE_SET;
    else if (this->display_mode_internal_ == display_options::ACT) display_mode = protocol::REPORT_DISP_MODE_ACT;
    else if (this->display_mode_internal_ == display_options::OUT) display_mode = protocol::REPORT_DISP_MODE_OUT;
    else display_mode = protocol::REPORT_DISP_MODE_AUTO;
  } else {
    display_mode = protocol::REPORT_DISP_MODE_AUTO;
    this->display_power_internal_ = true;
  }

  packet[protocol::REPORT_DISP_MODE_BYTE] |= (display_mode << protocol::REPORT_DISP_MODE_POS);
  if (this->display_power_internal_) packet[protocol::REPORT_DISP_ON_BYTE] |= protocol::REPORT_DISP_ON_MASK;

  // DISPLAY UNIT
  if (this->display_unit_state_ == display_unit_options::DEGF) {
    packet[protocol::REPORT_DISP_F_BYTE] |= protocol::REPORT_DISP_F_MASK;
  }

  // PLASMA
  if (this->plasma_state_) {
    packet[protocol::REPORT_PLASMA1_BYTE] |= protocol::REPORT_PLASMA1_MASK;
    packet[protocol::REPORT_PLASMA2_BYTE] |= protocol::REPORT_PLASMA2_MASK;
  }

  // SLEEP
  if (this->sleep_state_) packet[protocol::REPORT_SLEEP_BYTE] |= protocol::REPORT_SLEEP_MASK;

  // XFAN
  if (this->xfan_state_) packet[protocol::REPORT_XFAN_BYTE] |= protocol::REPORT_XFAN_MASK;

  // SAVE
  if (this->save_state_) packet[protocol::REPORT_SAVE_BYTE] |= protocol::REPORT_SAVE_MASK;

  // Command + length
  packet.insert(packet.begin(), protocol::CMD_OUT_PARAMS_SET);
  packet.insert(packet.begin(), protocol::SET_PACKET_LEN + 2);

  // checksum
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < packet.size(); i++) checksum += packet[i];
  packet.push_back(checksum);

  // SYNC
  packet.insert(packet.begin(), protocol::SYNC);
  packet.insert(packet.begin(), protocol::SYNC);

  this->last_packet_sent_ = millis();
  this->wait_response_ = true;
  write_array(packet);
  log_packet(packet, true);

  switch (this->update_) {
    case ACUpdate::NoUpdate: break;
    case ACUpdate::UpdateStart: this->update_ = ACUpdate::UpdateClear; break;
    case ACUpdate::UpdateClear: this->update_ = ACUpdate::NoUpdate; break;
    default: this->update_ = ACUpdate::NoUpdate; break;
  }
}

/*
 * Packet handling
 */

bool SinclairACCNT::verify_packet() {
  if (this->serialProcess_.data.size() < 5) {
    ESP_LOGW(TAG, "Dropping invalid packet (length)");
    return false;
  }

  bool commandAllowed = false;
  for (uint8_t packet : allowedPackets) {
    if (this->serialProcess_.data[3] == packet) {
      commandAllowed = true;
      break;
    }
  }
  if (!commandAllowed) {
    ESP_LOGW(TAG, "Dropping invalid packet (command [%02X] not allowed)", this->serialProcess_.data[3]);
    return false;
  }

  uint8_t checksum = 0;
  for (uint8_t i = 2; i < this->serialProcess_.data.size() - 1; i++) {
    checksum += this->serialProcess_.data[i];
  }
  if (checksum != this->serialProcess_.data[this->serialProcess_.data.size() - 1]) {
    ESP_LOGD(TAG, "Dropping invalid packet (checksum)");
    return false;
  }

  return true;
}

void SinclairACCNT::handle_packet() {
  if (this->serialProcess_.data[3] == protocol::CMD_IN_UNIT_REPORT) {
    this->serialProcess_.data.erase(this->serialProcess_.data.begin(), this->serialProcess_.data.begin() + 4);
    this->serialProcess_.data.pop_back();
    this->processUnitReport();
    this->publish_state();
  } else {
    ESP_LOGD(TAG, "Received unknown packet");
  }
}

/*
 * This decodes frame received from AC Unit
 */
bool SinclairACCNT::processUnitReport() {
  bool hasChanged = false;

  climate::ClimateMode newMode = determine_mode();
  if (this->mode != newMode) hasChanged = true;
  this->mode = newMode;

  const char *newFanMode = determine_fan_mode();
  if (this->has_custom_fan_mode()) {
    if (strcmp(this->get_custom_fan_mode(), newFanMode) != 0) hasChanged = true;
  } else {
    hasChanged = true;
  }
  this->set_custom_fan_mode_(newFanMode);

  float newTargetTemperature =
      (float) (((this->serialProcess_.data[protocol::REPORT_TEMP_SET_BYTE] & protocol::REPORT_TEMP_SET_MASK) >>
                protocol::REPORT_TEMP_SET_POS) +
               protocol::REPORT_TEMP_SET_OFF);
  if (this->target_temperature != newTargetTemperature) hasChanged = true;
  this->update_target_temperature(newTargetTemperature);

  if (this->current_temperature_sensor_ == nullptr) {
    float newCurrentTemperature =
        (float) (((this->serialProcess_.data[protocol::REPORT_TEMP_ACT_BYTE] & protocol::REPORT_TEMP_ACT_MASK) >>
                  protocol::REPORT_TEMP_ACT_POS) -
                 protocol::REPORT_TEMP_ACT_OFF) /
        protocol::REPORT_TEMP_ACT_DIV;
    if (this->current_temperature != newCurrentTemperature) hasChanged = true;
    this->update_current_temperature(newCurrentTemperature);
  }

  std::string verticalSwing = determine_vertical_swing();
  this->update_swing_vertical(verticalSwing);

  climate::ClimateSwingMode newSwingMode;
  if (verticalSwing == vertical_swing_options::FULL)
    newSwingMode = climate::CLIMATE_SWING_VERTICAL;
  else
    newSwingMode = climate::CLIMATE_SWING_OFF;

  if (this->swing_mode != newSwingMode) hasChanged = true;
  this->swing_mode = newSwingMode;

  // Don't override user-requested display changes while waiting for ACK
  if (!(this->pending_mask_ & PEND_DISPLAY)) {
    this->update_display(determine_display());
  }

  if (!(this->pending_mask_ & PEND_DISP_UNIT)) {
    this->update_display_unit(determine_display_unit());
  }


  this->update_plasma(determine_plasma());
  this->update_sleep(determine_sleep());
  this->update_xfan(determine_xfan());
  this->update_save(determine_save());

  return hasChanged;
}

climate::ClimateMode SinclairACCNT::determine_mode() {
  uint8_t mode = (this->serialProcess_.data[protocol::REPORT_MODE_BYTE] & protocol::REPORT_MODE_MASK) >> protocol::REPORT_MODE_POS;
  this->power_internal_ = (this->serialProcess_.data[protocol::REPORT_PWR_BYTE] & protocol::REPORT_PWR_MASK) != 0;

  switch (mode) {
    case protocol::REPORT_MODE_AUTO: this->mode_internal_ = climate::CLIMATE_MODE_AUTO; break;
    case protocol::REPORT_MODE_COOL: this->mode_internal_ = climate::CLIMATE_MODE_COOL; break;
    case protocol::REPORT_MODE_DRY:  this->mode_internal_ = climate::CLIMATE_MODE_DRY; break;
    case protocol::REPORT_MODE_FAN:  this->mode_internal_ = climate::CLIMATE_MODE_FAN_ONLY; break;
    case protocol::REPORT_MODE_HEAT: this->mode_internal_ = climate::CLIMATE_MODE_HEAT; break;
    default:
      ESP_LOGW(TAG, "Received unknown climate mode");
      this->mode_internal_ = climate::CLIMATE_MODE_OFF;
      break;
  }

  return this->power_internal_ ? this->mode_internal_ : climate::CLIMATE_MODE_OFF;
}

const char *SinclairACCNT::determine_fan_mode() {
  uint8_t fanSpeed1 = (this->serialProcess_.data[protocol::REPORT_FAN_SPD1_BYTE] & protocol::REPORT_FAN_SPD1_MASK) >> protocol::REPORT_FAN_SPD1_POS;
  uint8_t fanSpeed2 = (this->serialProcess_.data[protocol::REPORT_FAN_SPD2_BYTE] & protocol::REPORT_FAN_SPD2_MASK) >> protocol::REPORT_FAN_SPD2_POS;
  bool fanTurbo = (this->serialProcess_.data[protocol::REPORT_FAN_TURBO_BYTE] & protocol::REPORT_FAN_TURBO_MASK) != 0;

  if (fanTurbo) return fan_modes::FAN_TURBO;
  if (fanSpeed2 == 0) return fan_modes::FAN_AUTO;
  if (fanSpeed2 == 1) return fan_modes::FAN_LOW;
  if (fanSpeed2 == 2) return fan_modes::FAN_MED;
  if (fanSpeed2 == 3) return fan_modes::FAN_HIGH;

  ESP_LOGW(TAG, "Received unknown fan mode (fanSpeed1=%u fanSpeed2=%u turbo=%u)",
           fanSpeed1, fanSpeed2, (uint8_t) fanTurbo);
  return fan_modes::FAN_AUTO;
}

std::string SinclairACCNT::determine_vertical_swing() {
  uint8_t mode = (this->serialProcess_.data[protocol::REPORT_VSWING_BYTE] & protocol::REPORT_VSWING_MASK) >> protocol::REPORT_VSWING_POS;

  switch (mode) {
    case protocol::REPORT_VSWING_OFF:   return vertical_swing_options::OFF;
    case protocol::REPORT_VSWING_FULL:  return vertical_swing_options::FULL;
    case protocol::REPORT_VSWING_DOWN:  return vertical_swing_options::DOWN;
    case protocol::REPORT_VSWING_MIDD:  return vertical_swing_options::MIDD;
    case protocol::REPORT_VSWING_MID:   return vertical_swing_options::MID;
    case protocol::REPORT_VSWING_MIDU:  return vertical_swing_options::MIDU;
    case protocol::REPORT_VSWING_UP:    return vertical_swing_options::UP;
    case protocol::REPORT_VSWING_CDOWN: return vertical_swing_options::CDOWN;
    case protocol::REPORT_VSWING_CMIDD: return vertical_swing_options::CMIDD;
    case protocol::REPORT_VSWING_CMID:  return vertical_swing_options::CMID;
    case protocol::REPORT_VSWING_CMIDU: return vertical_swing_options::CMIDU;
    case protocol::REPORT_VSWING_CUP:   return vertical_swing_options::CUP;
    default:
      ESP_LOGW(TAG, "Received unknown vertical swing mode");
      return vertical_swing_options::OFF;
  }
}

std::string SinclairACCNT::determine_display() {
  uint8_t mode = (this->serialProcess_.data[protocol::REPORT_DISP_MODE_BYTE] & protocol::REPORT_DISP_MODE_MASK) >> protocol::REPORT_DISP_MODE_POS;
  this->display_power_internal_ = (this->serialProcess_.data[protocol::REPORT_DISP_ON_BYTE] & protocol::REPORT_DISP_ON_MASK);

  switch (mode) {
    case protocol::REPORT_DISP_MODE_AUTO: this->display_mode_internal_ = display_options::AUTO; break;
    case protocol::REPORT_DISP_MODE_SET:  this->display_mode_internal_ = display_options::SET; break;
    case protocol::REPORT_DISP_MODE_ACT:  this->display_mode_internal_ = display_options::ACT; break;
    case protocol::REPORT_DISP_MODE_OUT:  this->display_mode_internal_ = display_options::OUT; break;
    default:
      ESP_LOGW(TAG, "Received unknown display mode");
      this->display_mode_internal_ = display_options::AUTO;
      break;
  }

  return this->display_power_internal_ ? this->display_mode_internal_ : display_options::OFF;
}

std::string SinclairACCNT::determine_display_unit() {
  if (this->serialProcess_.data[protocol::REPORT_DISP_F_BYTE] & protocol::REPORT_DISP_F_MASK)
    return display_unit_options::DEGF;
  return display_unit_options::DEGC;
}

bool SinclairACCNT::determine_plasma() {
  bool plasma1 = (this->serialProcess_.data[protocol::REPORT_PLASMA1_BYTE] & protocol::REPORT_PLASMA1_MASK) != 0;
  bool plasma2 = (this->serialProcess_.data[protocol::REPORT_PLASMA2_BYTE] & protocol::REPORT_PLASMA2_MASK) != 0;
  return plasma1 || plasma2;
}

bool SinclairACCNT::determine_sleep() { return (this->serialProcess_.data[protocol::REPORT_SLEEP_BYTE] & protocol::REPORT_SLEEP_MASK) != 0; }
bool SinclairACCNT::determine_xfan()  { return (this->serialProcess_.data[protocol::REPORT_XFAN_BYTE] & protocol::REPORT_XFAN_MASK) != 0; }
bool SinclairACCNT::determine_save()  { return (this->serialProcess_.data[protocol::REPORT_SAVE_BYTE] & protocol::REPORT_SAVE_MASK) != 0; }

/*
 * Sensor handling
 */

void SinclairACCNT::on_vertical_swing_change(const std::string &swing) {
  if (this->state_ != ACState::Ready) return;

  ESP_LOGD(TAG, "Setting vertical swing position");
  this->update_ = ACUpdate::UpdateStart;
  this->vertical_swing_state_ = swing;

  uint8_t vs = protocol::REPORT_VSWING_OFF;
  if (swing == vertical_swing_options::OFF) vs = protocol::REPORT_VSWING_OFF;
  else if (swing == vertical_swing_options::FULL) vs = protocol::REPORT_VSWING_FULL;
  else if (swing == vertical_swing_options::DOWN) vs = protocol::REPORT_VSWING_DOWN;
  else if (swing == vertical_swing_options::MIDD) vs = protocol::REPORT_VSWING_MIDD;
  else if (swing == vertical_swing_options::MID) vs = protocol::REPORT_VSWING_MID;
  else if (swing == vertical_swing_options::MIDU) vs = protocol::REPORT_VSWING_MIDU;
  else if (swing == vertical_swing_options::UP) vs = protocol::REPORT_VSWING_UP;
  else if (swing == vertical_swing_options::CDOWN) vs = protocol::REPORT_VSWING_CDOWN;
  else if (swing == vertical_swing_options::CMIDD) vs = protocol::REPORT_VSWING_CMIDD;
  else if (swing == vertical_swing_options::CMID) vs = protocol::REPORT_VSWING_CMID;
  else if (swing == vertical_swing_options::CMIDU) vs = protocol::REPORT_VSWING_CMIDU;
  else if (swing == vertical_swing_options::CUP) vs = protocol::REPORT_VSWING_CUP;

  this->pend_vswing_ = vs;
  this->arm_pending_(PEND_VSWING);
}

void SinclairACCNT::on_display_change(const std::string &display) {
  if (this->state_ != ACState::Ready) return;

  ESP_LOGD(TAG, "Setting display mode");
  this->update_ = ACUpdate::UpdateStart;
  this->display_state_ = display;

  if (display == display_options::OFF) {
    this->pend_disp_on_ = false;
    if (this->display_mode_internal_ == display_options::SET) this->pend_disp_mode_ = protocol::REPORT_DISP_MODE_SET;
    else if (this->display_mode_internal_ == display_options::ACT) this->pend_disp_mode_ = protocol::REPORT_DISP_MODE_ACT;
    else if (this->display_mode_internal_ == display_options::OUT) this->pend_disp_mode_ = protocol::REPORT_DISP_MODE_OUT;
    else this->pend_disp_mode_ = protocol::REPORT_DISP_MODE_AUTO;
  } else {
    this->pend_disp_on_ = true;
    if (display == display_options::SET) this->pend_disp_mode_ = protocol::REPORT_DISP_MODE_SET;
    else if (display == display_options::ACT) this->pend_disp_mode_ = protocol::REPORT_DISP_MODE_ACT;
    else if (display == display_options::OUT) this->pend_disp_mode_ = protocol::REPORT_DISP_MODE_OUT;
    else this->pend_disp_mode_ = protocol::REPORT_DISP_MODE_AUTO;
  }

  this->arm_pending_(PEND_DISPLAY);
}

void SinclairACCNT::on_display_unit_change(const std::string &display_unit) {
  if (this->state_ != ACState::Ready) return;

  ESP_LOGD(TAG, "Setting display unit");
  this->update_ = ACUpdate::UpdateStart;
  this->display_unit_state_ = display_unit;

  this->pend_disp_f_ = (display_unit == display_unit_options::DEGF);
  this->arm_pending_(PEND_DISP_UNIT);
}

void SinclairACCNT::on_plasma_change(bool plasma) {
  if (this->state_ != ACState::Ready) return;

  ESP_LOGD(TAG, "Setting plasma");
  this->update_ = ACUpdate::UpdateStart;
  this->plasma_state_ = plasma;

  this->pend_plasma_ = plasma;
  this->arm_pending_(PEND_PLASMA);
}

void SinclairACCNT::on_sleep_change(bool sleep) {
  if (this->state_ != ACState::Ready) return;

  ESP_LOGD(TAG, "Setting sleep");
  this->update_ = ACUpdate::UpdateStart;
  this->sleep_state_ = sleep;

  this->pend_sleep_ = sleep;
  this->arm_pending_(PEND_SLEEP);
}

void SinclairACCNT::on_xfan_change(bool xfan) {
  if (this->state_ != ACState::Ready) return;

  ESP_LOGD(TAG, "Setting xfan");
  this->update_ = ACUpdate::UpdateStart;
  this->xfan_state_ = xfan;

  this->pend_xfan_ = xfan;
  this->arm_pending_(PEND_XFAN);
}

void SinclairACCNT::on_save_change(bool save) {
  if (this->state_ != ACState::Ready) return;

  ESP_LOGD(TAG, "Setting save");
  this->update_ = ACUpdate::UpdateStart;
  this->save_state_ = save;

  this->pend_save_ = save;
  this->arm_pending_(PEND_SAVE);
}

}  // namespace CNT
}  // namespace sinclair_ac
}  // namespace esphome
