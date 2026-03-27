#pragma once

#include "ups_hid.h"

namespace esphome {
namespace ups_hid {

/**
 * @brief CyberPower HID Protocol Implementation
 *
 * Supports CyberPower models that use the interrupt IN endpoint exclusively
 * for data reporting (e.g. CP1500EPFCLCD and similar).
 *
 * These devices push HID reports via interrupt IN rather than responding to
 * HID GET_REPORT control requests for live data.  The protocol therefore
 * starts the interrupt IN endpoint during initialize() and drains it each
 * time read_data() is called.
 *
 * Report IDs observed on the interrupt IN endpoint
 * ------------------------------------------------
 *   0x20 (4 bytes) : battery %  +  battery voltage (decivolts)
 *   0x21 (3 bytes) : runtime to empty in seconds (16-bit LE)
 *   0x22 (2 bytes) : load percentage
 *   0x23 (5 bytes) : input voltage + output voltage (16-bit LE each, in volts)
 *   0x25 (2 bytes) : unknown small counter (logged, not mapped)
 *   0x28 (2 bytes) : status flags (currently always 0x00)
 *   0x29 (2 bytes) : on-battery flags  (0x00 = on AC, 0x03 = on battery)
 *   0x80 (2 bytes) : constant config byte
 *   0x82 (3 bytes) : nominal real power (16-bit LE, watts)
 *   0x85 (2 bytes) : constant (unused)
 *   0x86 (3 bytes) : high-voltage transfer limit (16-bit LE; 0xFFFF = disabled)
 *   0x87 (3 bytes) : low-voltage transfer limit  (16-bit LE; 0xFFFF = disabled)
 *   0x88 (3 bytes) : nominal input voltage (16-bit LE, volts)
 *
 * Feature reports (via HID SET_REPORT) are still used for beeper and test
 * commands; their report IDs come from the device's HID descriptor and may
 * differ from the interrupt IDs above.
 */

class CyberPowerProtocol : public UpsProtocolBase {
 public:
  CyberPowerProtocol(UpsHidComponent *parent) : UpsProtocolBase(parent) {}

  bool detect() override;
  bool initialize() override;
  bool read_data(UpsData &data) override;
  DeviceInfo::DetectedProtocol get_protocol_type() const override {
    return DeviceInfo::PROTOCOL_CYBERPOWER_HID;
  }
  std::string get_protocol_name() const override { return "CyberPower HID"; }

  // Beeper control (uses HID SET_REPORT feature report)
  bool beeper_enable() override;
  bool beeper_disable() override;
  bool beeper_mute() override;
  bool beeper_test() override;

  // Battery / UPS test (uses HID SET_REPORT feature report)
  bool start_battery_test_quick() override;
  bool start_battery_test_deep() override;
  bool stop_battery_test() override;
  bool start_ups_test() override;
  bool stop_ups_test() override;

  // Timer polling for real-time countdown
  bool read_timer_data(UpsData &data) override;

  // Delay configuration
  bool set_shutdown_delay(int seconds) override;
  bool set_start_delay(int seconds) override;
  bool set_reboot_delay(int seconds) override;

 private:
  // ── Interrupt IN report IDs ──────────────────────────────────────────────
  static const uint8_t INT_BATTERY_STATUS_REPORT_ID  = 0x20; // batt% + voltage
  static const uint8_t INT_RUNTIME_REPORT_ID          = 0x21; // runtime secs
  static const uint8_t INT_LOAD_REPORT_ID             = 0x22; // load %
  static const uint8_t INT_VOLTAGE_REPORT_ID          = 0x23; // in/out voltage
  static const uint8_t INT_UNKNOWN_25_REPORT_ID       = 0x25; // unknown
  static const uint8_t INT_FLAGS_28_REPORT_ID         = 0x28; // status flags
  static const uint8_t INT_ON_BATTERY_REPORT_ID       = 0x29; // on-battery state
  static const uint8_t INT_CONFIG_80_REPORT_ID        = 0x80; // constant config
  static const uint8_t INT_NOMINAL_POWER_REPORT_ID    = 0x82; // nominal power W
  static const uint8_t INT_CONFIG_85_REPORT_ID        = 0x85; // unused constant
  static const uint8_t INT_XFER_HIGH_REPORT_ID        = 0x86; // high xfer limit
  static const uint8_t INT_XFER_LOW_REPORT_ID         = 0x87; // low  xfer limit
  static const uint8_t INT_NOMINAL_VOLT_REPORT_ID     = 0x88; // nominal in-volt

  // ── Feature report IDs for commands ─────────────────────────────────────
  static const uint8_t BEEPER_STATUS_REPORT_ID    = 0x0c;
  static const uint8_t TEST_RESULT_REPORT_ID      = 0x14;
  static const uint8_t DELAY_SHUTDOWN_REPORT_ID   = 0x15;
  static const uint8_t DELAY_START_REPORT_ID      = 0x16;

  // ── State ────────────────────────────────────────────────────────────────
  bool interrupt_started_{false};
  // String descriptors are fetched once and cached here so that
  // ups_data_.reset() on every update cycle does not cause repeated USB
  // GET_STRING_DESCRIPTOR requests (and repeated serial-not-found warnings).
  bool strings_initialized_{false};
  std::string cached_manufacturer_;
  std::string cached_model_;
  std::string cached_serial_;
  std::string cached_firmware_;

  // Power status (online / on-battery) is cached across read cycles.
  // ups_data_.reset() clears power.status to "" each cycle; report 0x29 is
  // only guaranteed to appear within MAX_REPORTS_PER_READ items per cycle.
  // Caching the last-seen value ensures the binary sensors reflect the true
  // last-known state even when 0x29 falls outside the drain window.
  std::string cached_power_status_;

  // ── Interrupt report parsers ─────────────────────────────────────────────
  void parse_interrupt_report(const uint8_t* data, size_t len, UpsData &out);
  void parse_int_battery_status(const uint8_t* data, size_t len, UpsData &out);
  void parse_int_runtime(const uint8_t* data, size_t len, UpsData &out);
  void parse_int_load(const uint8_t* data, size_t len, UpsData &out);
  void parse_int_voltage(const uint8_t* data, size_t len, UpsData &out);
  void parse_int_on_battery(const uint8_t* data, size_t len, UpsData &out);
  void parse_int_nominal_power(const uint8_t* data, size_t len, UpsData &out);
  void parse_int_xfer_limits(uint8_t report_id, const uint8_t* data, size_t len, UpsData &out);
  void parse_int_nominal_voltage(const uint8_t* data, size_t len, UpsData &out);

  // ── Feature-report helper (still used for commands / detection fallback) ─
  struct HidReport {
    uint8_t report_id;
    std::vector<uint8_t> data;
    HidReport() : report_id(0) {}
  };
  bool read_hid_feature_report(uint8_t report_id, HidReport &report);

  // ── String helpers ───────────────────────────────────────────────────────
  std::string clean_firmware_string(const std::string &raw);
  void read_device_strings(UpsData &data);
};

}  // namespace ups_hid
}  // namespace esphome
