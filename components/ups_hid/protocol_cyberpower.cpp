#include "protocol_cyberpower.h"
#include "ups_hid.h"
#include "constants_hid.h"
#include "constants_ups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "esp_err.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cctype>
#include <cmath>

namespace esphome {
namespace ups_hid {

static const char *const CP_TAG = "ups_hid.cyberpower_hid";

// ── Interrupt report drain settings ──────────────────────────────────────────
// How long to wait for the FIRST report in each read_data() call (ms).
// The transport auto-starts interrupt IN at device-connect time, so reports
// are already queued long before read_data() is first called.  1500 ms is
// a generous safety margin for slow or reconnected devices.
static const uint32_t FIRST_REPORT_TIMEOUT_MS  = 1500;
// How long to wait for ADDITIONAL reports after the first (ms).
// Short so we drain the whole burst without blocking too long.
static const uint32_t NEXT_REPORT_TIMEOUT_MS   = 50;
// How many consecutive interrupts to process in one read_data() call.
static const int      MAX_REPORTS_PER_READ      = 32;

// ─────────────────────────────────────────────────────────────────────────────
// detect()
// ─────────────────────────────────────────────────────────────────────────────
bool CyberPowerProtocol::detect() {
  ESP_LOGD(CP_TAG, "Detecting CyberPower HID (interrupt IN) protocol");

  if (!parent_->is_device_connected()) {
    ESP_LOGD(CP_TAG, "Device not connected, skipping protocol detection");
    return false;
  }

  // Interrupt IN was already started by the transport at device-connect time;
  // this call is a no-op if active, or restarts it if somehow stopped.
  esp_err_t ret = parent_->start_interrupt_in();
  if (ret != ESP_OK) {
    ESP_LOGD(CP_TAG, "start_interrupt_in() returned %s — not an interrupt device",
             esp_err_to_name(ret));
    return false;
  }

  // Give the device up to 2 seconds to send at least one report.
  static const uint32_t DETECT_TIMEOUT_MS = 2000;
  uint8_t buf[8];
  size_t  len = 0;
  bool    found = false;

  uint32_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(DETECT_TIMEOUT_MS);
  while (xTaskGetTickCount() < deadline) {
    len = 0;
    if (!parent_->read_interrupt_report(buf, &len, 200 /*ms*/)) continue;
    if (len == 0) continue;

    uint8_t id = buf[0];
    // Any report in the known CyberPower interrupt range is a positive match.
    if ((id >= 0x20 && id <= 0x29) || (id >= 0x80 && id <= 0x88)) {
      ESP_LOGI(CP_TAG,
               "CyberPower interrupt protocol detected via report 0x%02X (%zu bytes)",
               id, len);
      found = true;
      break;
    }
  }

  if (!found) {
    ESP_LOGD(CP_TAG, "No CyberPower interrupt reports received within timeout");
    parent_->stop_interrupt_in();
    interrupt_started_ = false;
    return false;
  }

  // Keep the interrupt IN running; initialize() won't need to start it again.
  interrupt_started_ = true;
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// initialize()
// ─────────────────────────────────────────────────────────────────────────────
bool CyberPowerProtocol::initialize() {
  ESP_LOGI(CP_TAG, "Initializing CyberPower HID (interrupt IN) protocol");

  if (!interrupt_started_) {
    esp_err_t ret = parent_->start_interrupt_in();
    if (ret != ESP_OK) {
      ESP_LOGE(CP_TAG, "Failed to start interrupt IN: %s", esp_err_to_name(ret));
      return false;
    }
    interrupt_started_ = true;
  }

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// read_data()
// ─────────────────────────────────────────────────────────────────────────────
bool CyberPowerProtocol::read_data(UpsData &data) {
  ESP_LOGD(CP_TAG, "Reading CyberPower HID data (interrupt IN)");

  if (!parent_->is_device_connected()) {
    ESP_LOGV(CP_TAG, "Device not connected");
    return false;
  }

  // Ensure interrupt IN is running.  The transport auto-starts it at device-
  // connect time, so this is normally a no-op.  Calling unconditionally also
  // handles the reconnect case where interrupt_started_ is stale-true but the
  // queue was torn down during the previous disconnect.
  {
    esp_err_t ret = parent_->start_interrupt_in();
    if (ret != ESP_OK) {
      ESP_LOGE(CP_TAG, "Failed to (re)start interrupt IN: %s", esp_err_to_name(ret));
      return false;
    }
    interrupt_started_ = true;
  }

  // Restore last-known power status before draining the queue so that
  // the binary sensors show the correct state even in cycles where report
  // 0x29 falls outside the MAX_REPORTS_PER_READ drain window.
  if (!cached_power_status_.empty()) {
    data.power.status = cached_power_status_;
  }

  bool success = false;
  int  reports_processed = 0;
  uint8_t buf[8];
  size_t  len = 0;

  // Wait for the first report with a longer timeout, then drain the queue.
  uint32_t timeout = FIRST_REPORT_TIMEOUT_MS;
  for (int i = 0; i < MAX_REPORTS_PER_READ; i++) {
    len = 0;
    if (!parent_->read_interrupt_report(buf, &len, timeout)) break;
    if (len == 0) break;

    parse_interrupt_report(buf, len, data);
    reports_processed++;
    success = true;
    timeout = NEXT_REPORT_TIMEOUT_MS;  // short timeout for subsequent reports
  }

  if (!success) {
    // Queue was empty within the timeout – expected after a burst is fully
    // drained, or during a quiet polling interval.  Not a real error.
    ESP_LOGV(CP_TAG, "No interrupt reports received from UPS (queue empty)");
    return false;
  }

  ESP_LOGD(CP_TAG, "Processed %d interrupt reports", reports_processed);

  // Read USB string descriptors once we have confirmed communication.
  read_device_strings(data);

  // Derive timer values from config delays (negative = not active).
  data.test.timer_shutdown = -data.config.delay_shutdown;
  data.test.timer_start    = -data.config.delay_start;
  data.test.timer_reboot   = defaults::REBOOT_TIMER_DEFAULT;

  if (data.test.ups_test_result.empty()) {
    data.test.ups_test_result = test::RESULT_NO_TEST;
  }

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// parse_interrupt_report() — dispatch to per-report parsers
// ─────────────────────────────────────────────────────────────────────────────
void CyberPowerProtocol::parse_interrupt_report(const uint8_t* data, size_t len,
                                                UpsData &out) {
  if (len == 0) return;
  uint8_t id = data[0];

  switch (id) {
    case INT_BATTERY_STATUS_REPORT_ID: parse_int_battery_status(data, len, out); break;
    case INT_RUNTIME_REPORT_ID:        parse_int_runtime(data, len, out);        break;
    case INT_LOAD_REPORT_ID:           parse_int_load(data, len, out);           break;
    case INT_VOLTAGE_REPORT_ID:        parse_int_voltage(data, len, out);        break;
    case INT_ON_BATTERY_REPORT_ID:     parse_int_on_battery(data, len, out);     break;
    case INT_NOMINAL_POWER_REPORT_ID:  parse_int_nominal_power(data, len, out);  break;
    case INT_XFER_HIGH_REPORT_ID:      // fall-through
    case INT_XFER_LOW_REPORT_ID:       parse_int_xfer_limits(id, data, len, out); break;
    case INT_NOMINAL_VOLT_REPORT_ID:   parse_int_nominal_voltage(data, len, out); break;
    case INT_UNKNOWN_25_REPORT_ID:
      ESP_LOGV(CP_TAG, "Report 0x25: value=%d (unmapped)", len >= 2 ? data[1] : 0);
      break;
    case INT_FLAGS_28_REPORT_ID:
    case INT_CONFIG_80_REPORT_ID:
    case INT_CONFIG_85_REPORT_ID:
      ESP_LOGV(CP_TAG, "Report 0x%02X: constant/unused", id);
      break;
    default:
      ESP_LOGV(CP_TAG, "Unknown interrupt report 0x%02X (%zu bytes)", id, len);
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Interrupt report parsers
// ─────────────────────────────────────────────────────────────────────────────

// 0x20: [id, batt_pct, batt_voltage*10, 0x00]
void CyberPowerProtocol::parse_int_battery_status(const uint8_t* data, size_t len,
                                                   UpsData &out) {
  if (len < 3) {
    ESP_LOGW(CP_TAG, "Report 0x20 too short (%zu)", len);
    return;
  }

  uint8_t batt_pct = data[1];
  uint8_t volt_raw  = data[2];

  out.battery.level   = static_cast<float>(batt_pct > battery::MAX_LEVEL_PERCENT
                                            ? battery::MAX_LEVEL_PERCENT : batt_pct);
  out.battery.voltage = static_cast<float>(volt_raw) / battery::VOLTAGE_SCALE_FACTOR;

  ESP_LOGD(CP_TAG, "Battery: %.0f%%  %.1fV (raw volt=0x%02X)",
           out.battery.level, out.battery.voltage, volt_raw);
}

// 0x21: [id, runtime_lo, runtime_hi]  — seconds, 16-bit LE
void CyberPowerProtocol::parse_int_runtime(const uint8_t* data, size_t len,
                                            UpsData &out) {
  if (len < 3) {
    ESP_LOGW(CP_TAG, "Report 0x21 too short (%zu)", len);
    return;
  }

  uint16_t runtime_sec = static_cast<uint16_t>(data[1]) |
                         (static_cast<uint16_t>(data[2]) << 8);
  out.battery.runtime_minutes = static_cast<float>(runtime_sec) / 60.0f;

  ESP_LOGD(CP_TAG, "Runtime: %.1f min (%u sec)", out.battery.runtime_minutes, runtime_sec);
}

// 0x22: [id, load_pct]
void CyberPowerProtocol::parse_int_load(const uint8_t* data, size_t len,
                                         UpsData &out) {
  if (len < 2) {
    ESP_LOGW(CP_TAG, "Report 0x22 too short (%zu)", len);
    return;
  }

  out.power.load_percent = static_cast<float>(data[1]);
  ESP_LOGD(CP_TAG, "Load: %.0f%%", out.power.load_percent);
}

// 0x23: [id, in_lo, in_hi, out_lo, out_hi]  — volts, 16-bit LE
void CyberPowerProtocol::parse_int_voltage(const uint8_t* data, size_t len,
                                            UpsData &out) {
  if (len < 5) {
    ESP_LOGW(CP_TAG, "Report 0x23 too short (%zu)", len);
    return;
  }

  uint16_t in_v  = static_cast<uint16_t>(data[1]) | (static_cast<uint16_t>(data[2]) << 8);
  uint16_t out_v = static_cast<uint16_t>(data[3]) | (static_cast<uint16_t>(data[4]) << 8);

  out.power.input_voltage  = static_cast<float>(in_v);
  out.power.output_voltage = static_cast<float>(out_v);

  ESP_LOGD(CP_TAG, "Voltage: in=%.0fV  out=%.0fV", out.power.input_voltage,
           out.power.output_voltage);
}

// 0x29: [id, flags]
//   0x00 = on AC (not discharging)
//   0x01 = discharging
//   0x02 = AC absent
//   0x03 = discharging + AC absent (typical on-battery state)
void CyberPowerProtocol::parse_int_on_battery(const uint8_t* data, size_t len,
                                               UpsData &out) {
  if (len < 2) {
    ESP_LOGW(CP_TAG, "Report 0x29 too short (%zu)", len);
    return;
  }

  uint8_t flags        = data[1];
  bool    discharging  = (flags & 0x01) != 0;
  bool    ac_absent    = (flags & 0x02) != 0;
  bool    on_battery   = discharging || ac_absent;

  if (on_battery) {
    out.power.status     = status::ON_BATTERY;
    out.battery.status   = battery_status::DISCHARGING;
    if (!ac_absent) {
      // AC present but discharging — treat input voltage as nominal fallback
      out.power.input_voltage = parent_->get_fallback_nominal_voltage();
    } else {
      out.power.input_voltage = NAN;
    }
  } else {
    out.power.status = status::ONLINE;
    // Battery charging state will be inferred from battery % in next report;
    // default to NORMAL when on AC.
    if (out.battery.status.empty() ||
        out.battery.status == battery_status::DISCHARGING) {
      out.battery.status = battery_status::NORMAL;
    }
  }

  // Keep cache in sync so future cycles that don't receive 0x29 still publish
  // the correct last-known state (see cached_power_status_ in header).
  cached_power_status_ = out.power.status;

  ESP_LOGD(CP_TAG, "OnBattery flags=0x%02X  discharging=%d  ac_absent=%d  status=%s",
           flags, discharging, ac_absent, out.power.status.c_str());
}

// 0x82: [id, power_lo, power_hi]  — watts, 16-bit LE
void CyberPowerProtocol::parse_int_nominal_power(const uint8_t* data, size_t len,
                                                  UpsData &out) {
  if (len < 3) return;
  uint16_t power = static_cast<uint16_t>(data[1]) | (static_cast<uint16_t>(data[2]) << 8);
  out.power.realpower_nominal = static_cast<float>(power);
  ESP_LOGD(CP_TAG, "Nominal power: %.0fW", out.power.realpower_nominal);
}

// 0x86 = high transfer limit, 0x87 = low transfer limit  [id, lo, hi] — 16-bit LE
// 0xFFFF means disabled/not configured.
void CyberPowerProtocol::parse_int_xfer_limits(uint8_t report_id, const uint8_t* data,
                                                size_t len, UpsData &out) {
  if (len < 3) return;
  uint16_t val = static_cast<uint16_t>(data[1]) | (static_cast<uint16_t>(data[2]) << 8);
  if (val == 0xFFFF) return;  // disabled

  if (report_id == INT_XFER_HIGH_REPORT_ID) {
    out.power.input_transfer_high = static_cast<float>(val);
    ESP_LOGD(CP_TAG, "Transfer high: %.0fV", out.power.input_transfer_high);
  } else {
    out.power.input_transfer_low = static_cast<float>(val);
    ESP_LOGD(CP_TAG, "Transfer low: %.0fV", out.power.input_transfer_low);
  }
}

// 0x88: [id, volt_lo, volt_hi]  — nominal input voltage, 16-bit LE
void CyberPowerProtocol::parse_int_nominal_voltage(const uint8_t* data, size_t len,
                                                    UpsData &out) {
  if (len < 3) return;
  uint16_t val = static_cast<uint16_t>(data[1]) | (static_cast<uint16_t>(data[2]) << 8);
  if (val == 0) return;
  out.power.input_voltage_nominal = static_cast<float>(val);
  ESP_LOGD(CP_TAG, "Nominal input voltage: %.0fV", out.power.input_voltage_nominal);
}

// ─────────────────────────────────────────────────────────────────────────────
// read_device_strings() — USB string descriptors (serial, firmware, model)
// ─────────────────────────────────────────────────────────────────────────────
void CyberPowerProtocol::read_device_strings(UpsData &data) {
  // ups_data_.reset() is called before every read_data(), so data.device.*
  // fields are always empty on entry.  Use the protocol-level cache so we only
  // issue USB GET_STRING_DESCRIPTOR requests once per device connection.
  if (strings_initialized_) {
    data.device.manufacturer    = cached_manufacturer_;
    data.device.model           = cached_model_;
    data.device.serial_number   = cached_serial_;
    data.device.firmware_version = cached_firmware_;
    return;
  }

  // Manufacturer: string descriptor index 3 ("CPS")
  {
    std::string s;
    if (parent_->usb_get_string_descriptor(3, s) == ESP_OK && !s.empty()) {
      cached_manufacturer_ = s;
      ESP_LOGI(CP_TAG, "Manufacturer: \"%s\"", s.c_str());
    }
  }

  // Model: string descriptor index 1
  {
    std::string s;
    if (parent_->usb_get_string_descriptor(1, s) == ESP_OK && !s.empty()) {
      cached_model_ = s;
      ESP_LOGI(CP_TAG, "Model: \"%s\"", s.c_str());
    }
  }

  // Serial number: string descriptor index 2 (not present on all models)
  {
    std::string s;
    if (parent_->usb_get_string_descriptor(2, s) == ESP_OK && !s.empty()) {
      cached_serial_ = s;
      ESP_LOGI(CP_TAG, "Serial: \"%s\"", s.c_str());
    } else {
      ESP_LOGD(CP_TAG, "Serial number not available via string descriptor 2");
    }
  }

  // Firmware: try string descriptor indices 5, 4, 6 in order
  for (uint8_t idx : {5u, 4u, 6u}) {
    std::string s;
    if (parent_->usb_get_string_descriptor(idx, s) == ESP_OK && !s.empty()) {
      std::string cleaned = clean_firmware_string(s);
      if (!cleaned.empty()) {
        cached_firmware_ = cleaned;
        ESP_LOGI(CP_TAG, "Firmware (str idx %d): \"%s\"", idx, cleaned.c_str());
        break;
      }
    }
  }

  strings_initialized_ = true;

  data.device.manufacturer    = cached_manufacturer_;
  data.device.model           = cached_model_;
  data.device.serial_number   = cached_serial_;
  data.device.firmware_version = cached_firmware_;
}

// ─────────────────────────────────────────────────────────────────────────────
// read_timer_data() — lightweight re-read of runtime/status for fast polling
// ─────────────────────────────────────────────────────────────────────────────
bool CyberPowerProtocol::read_timer_data(UpsData &data) {
  return read_data(data);
}

// ─────────────────────────────────────────────────────────────────────────────
// Feature-report helper (for command / control transfers)
// ─────────────────────────────────────────────────────────────────────────────
bool CyberPowerProtocol::read_hid_feature_report(uint8_t report_id, HidReport &report) {
  if (!parent_->is_device_connected()) return false;

  uint8_t buffer[limits::MAX_HID_REPORT_SIZE];
  size_t  buffer_len = sizeof(buffer);

  esp_err_t ret = parent_->hid_get_report(HID_REPORT_TYPE_FEATURE, report_id,
                                           buffer, &buffer_len,
                                           parent_->get_protocol_timeout());
  if (ret == ESP_OK && buffer_len > 0) {
    report.report_id = report_id;
    report.data.assign(buffer, buffer + buffer_len);
    return true;
  }

  ESP_LOGD(CP_TAG, "Feature report 0x%02X failed: %s", report_id, esp_err_to_name(ret));
  return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Beeper control  (HID SET_REPORT feature)
// ─────────────────────────────────────────────────────────────────────────────
bool CyberPowerProtocol::beeper_enable() {
  uint8_t d[2] = {BEEPER_STATUS_REPORT_ID, beeper::CONTROL_ENABLE};
  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE,
                                           BEEPER_STATUS_REPORT_ID, d, 2,
                                           parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(CP_TAG, "Beeper enabled");
    return true;
  }
  ESP_LOGW(CP_TAG, "Beeper enable failed: %s", esp_err_to_name(ret));
  return false;
}

bool CyberPowerProtocol::beeper_disable() {
  uint8_t d[2] = {BEEPER_STATUS_REPORT_ID, beeper::CONTROL_DISABLE};
  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE,
                                           BEEPER_STATUS_REPORT_ID, d, 2,
                                           parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(CP_TAG, "Beeper disabled");
    return true;
  }
  ESP_LOGW(CP_TAG, "Beeper disable failed: %s", esp_err_to_name(ret));
  return false;
}

bool CyberPowerProtocol::beeper_mute() {
  uint8_t d[2] = {BEEPER_STATUS_REPORT_ID, beeper::CONTROL_MUTE};
  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE,
                                           BEEPER_STATUS_REPORT_ID, d, 2,
                                           parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(CP_TAG, "Beeper muted");
    return true;
  }
  ESP_LOGW(CP_TAG, "Beeper mute failed: %s", esp_err_to_name(ret));
  return false;
}

bool CyberPowerProtocol::beeper_test() {
  HidReport report;
  if (!read_hid_feature_report(BEEPER_STATUS_REPORT_ID, report)) {
    ESP_LOGW(CP_TAG, "Cannot read current beeper state for test");
    return false;
  }
  uint8_t original = (report.data.size() >= 2) ? report.data[1] : beeper::CONTROL_ENABLE;

  if (!beeper_disable()) return false;
  vTaskDelay(pdMS_TO_TICKS(3000));
  if (!beeper_enable())  return false;
  vTaskDelay(pdMS_TO_TICKS(500));

  // Restore original state
  uint8_t d[2] = {BEEPER_STATUS_REPORT_ID, original};
  parent_->hid_set_report(HID_REPORT_TYPE_FEATURE, BEEPER_STATUS_REPORT_ID, d, 2,
                           parent_->get_protocol_timeout());
  ESP_LOGI(CP_TAG, "Beeper test complete");
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Battery / UPS tests  (HID SET_REPORT feature)
// ─────────────────────────────────────────────────────────────────────────────
bool CyberPowerProtocol::start_battery_test_quick() {
  uint8_t d[2] = {TEST_RESULT_REPORT_ID, test::COMMAND_QUICK};
  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE,
                                           TEST_RESULT_REPORT_ID, d, 2,
                                           parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(CP_TAG, "Quick battery test started");
    return true;
  }
  ESP_LOGW(CP_TAG, "Quick battery test failed: %s", esp_err_to_name(ret));
  return false;
}

bool CyberPowerProtocol::start_battery_test_deep() {
  uint8_t d[2] = {TEST_RESULT_REPORT_ID, test::COMMAND_DEEP};
  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE,
                                           TEST_RESULT_REPORT_ID, d, 2,
                                           parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(CP_TAG, "Deep battery test started");
    return true;
  }
  ESP_LOGW(CP_TAG, "Deep battery test failed: %s", esp_err_to_name(ret));
  return false;
}

bool CyberPowerProtocol::stop_battery_test() {
  uint8_t d[2] = {TEST_RESULT_REPORT_ID, test::COMMAND_ABORT};
  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE,
                                           TEST_RESULT_REPORT_ID, d, 2,
                                           parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(CP_TAG, "Battery test stopped");
    return true;
  }
  ESP_LOGW(CP_TAG, "Stop battery test failed: %s", esp_err_to_name(ret));
  return false;
}

bool CyberPowerProtocol::start_ups_test() {
  return start_battery_test_quick();
}

bool CyberPowerProtocol::stop_ups_test() {
  return stop_battery_test();
}

// ─────────────────────────────────────────────────────────────────────────────
// Delay configuration  (HID SET_REPORT feature)
// ─────────────────────────────────────────────────────────────────────────────
bool CyberPowerProtocol::set_shutdown_delay(int seconds) {
  int16_t val = static_cast<int16_t>(seconds);
  uint8_t d[3] = {DELAY_SHUTDOWN_REPORT_ID,
                  static_cast<uint8_t>(val & 0xFF),
                  static_cast<uint8_t>((val >> 8) & 0xFF)};
  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE,
                                           DELAY_SHUTDOWN_REPORT_ID, d, 3,
                                           parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(CP_TAG, "Shutdown delay set to %d s", seconds);
    return true;
  }
  ESP_LOGW(CP_TAG, "Set shutdown delay failed: %s", esp_err_to_name(ret));
  return false;
}

bool CyberPowerProtocol::set_start_delay(int seconds) {
  int16_t val = static_cast<int16_t>(seconds);
  uint8_t d[3] = {DELAY_START_REPORT_ID,
                  static_cast<uint8_t>(val & 0xFF),
                  static_cast<uint8_t>((val >> 8) & 0xFF)};
  esp_err_t ret = parent_->hid_set_report(HID_REPORT_TYPE_FEATURE,
                                           DELAY_START_REPORT_ID, d, 3,
                                           parent_->get_protocol_timeout());
  if (ret == ESP_OK) {
    ESP_LOGI(CP_TAG, "Start delay set to %d s", seconds);
    return true;
  }
  ESP_LOGW(CP_TAG, "Set start delay failed: %s", esp_err_to_name(ret));
  return false;
}

bool CyberPowerProtocol::set_reboot_delay(int seconds) {
  // CyberPower doesn't have a separate reboot-delay report; use shutdown delay.
  return set_shutdown_delay(seconds);
}

// ─────────────────────────────────────────────────────────────────────────────
// String helpers
// ─────────────────────────────────────────────────────────────────────────────
std::string CyberPowerProtocol::clean_firmware_string(const std::string &raw) {
  if (raw.empty()) return raw;

  std::string s = raw;
  // Keep alphanumeric, dot, dash, space
  s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c) {
    return !(std::isalnum(c) || c == '.' || c == '-' || c == ' ');
  }), s.end());

  // Trim leading/trailing whitespace
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());

  return s.empty() ? raw : s;
}

}  // namespace ups_hid
}  // namespace esphome

// ─────────────────────────────────────────────────────────────────────────────
// Factory registration
// ─────────────────────────────────────────────────────────────────────────────
#include "protocol_factory.h"

namespace esphome {
namespace ups_hid {

static std::unique_ptr<UpsProtocolBase> create_cyberpower_protocol(UpsHidComponent *parent) {
  return std::make_unique<CyberPowerProtocol>(parent);
}

}  // namespace ups_hid
}  // namespace esphome

// Register CyberPower protocol for vendor ID 0x0764
REGISTER_UPS_PROTOCOL_FOR_VENDOR(0x0764, cyberpower_hid_protocol,
                                 esphome::ups_hid::create_cyberpower_protocol,
                                 "CyberPower HID",
                                 "CyberPower HID interrupt IN protocol", 100);
