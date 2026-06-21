#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <cstdint>
#include <string>

namespace esphome {
namespace em24_meter {

// Emulates a Carlo Gavazzi EM24 Ethernet energy meter over Modbus TCP so that a
// Victron GX device (Cerbo/Venus OS) detects it as a native grid meter.
//
// Register layout and detection logic are taken from Victron's dbus-modbus-client
// (carlo_gavazzi.py):
//   - probe reads u16 @0x000B and expects a model id in 1648..1653
//   - /Ac/Power            s32 (low word first) @0x0028, scale x10  (W)
//   - /Ac/Frequency        u16 @0x0033, scale x10                   (Hz)
//   - /Ac/Energy/Forward   s32 @0x0034, scale x10                   (kWh)
//   - /Ac/Energy/Reverse   s32 @0x004E, scale x10                   (kWh)
//   - application reg      u16 @0xA000 must read 7 ("H")
//   - PhaseConfig          u16 @0x1002 (3 = 1P)
//   - per-phase L1: V@0x0000(x10) I@0x000C(x1000) P@0x0012(x10) Efwd@0x0040(x10)
class EM24Meter : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_port(uint16_t port) { this->port_ = port; }
  void set_unit_id(uint8_t unit_id) { this->unit_id_ = unit_id; }
  void set_phase_config(uint16_t v) { this->phase_config_ = v; }
  void set_serial(const std::string &s) { this->serial_ = s; }
  void set_invert_power(bool b) { this->invert_power_ = b; }

  void set_power_sensor_l1(sensor::Sensor *s) { this->power_L1 = s; }
  void set_power_sensor_l2(sensor::Sensor *s) { this->power_L2 = s; }
  void set_power_sensor_l3(sensor::Sensor *s) { this->power_L3 = s; }
  void set_import_sensor(sensor::Sensor *s) { this->import_ = s; }
  void set_export_sensor(sensor::Sensor *s) { this->export_ = s; }
  void set_voltage_sensor_l1(sensor::Sensor *s) { this->voltage_L1 = s; }
  void set_voltage_sensor_l2(sensor::Sensor *s) { this->voltage_L2 = s; }
  void set_voltage_sensor_l3(sensor::Sensor *s) { this->voltage_L3 = s; }
  void set_frequency_sensor(sensor::Sensor *s) { this->frequency_ = s; }

 protected:
  bool start_server_();
  void service_client_(int slot);
  // Builds a response into resp; returns response length, or 0 if no reply.
  size_t process_frame_(const uint8_t *req, size_t len, uint8_t *resp);
  uint16_t get_register_(uint16_t addr);

  // config
  uint16_t port_{502};
  uint8_t unit_id_{1};
  uint16_t phase_config_{3};  // 3 = "1P"
  std::string serial_{"EM24EMU0000001"};
  bool invert_power_{false};

  // sensors
  sensor::Sensor *power_L1{nullptr};
  sensor::Sensor *power_L2{nullptr};
  sensor::Sensor *power_L3{nullptr};
  sensor::Sensor *import_{nullptr};
  sensor::Sensor *export_{nullptr};
  sensor::Sensor *voltage_L1{nullptr};
  sensor::Sensor *voltage_L2{nullptr};
  sensor::Sensor *voltage_L3{nullptr};
  sensor::Sensor *frequency_{nullptr};

  // writable register Victron may set during init
  uint16_t app_reg_{7};  // application "H"

  // sockets
  static const int MAX_CLIENTS = 3;
  // Per-client receive buffer.  MBAP header (6 bytes) tells us the total frame
  // length, so we accumulate until we have a complete frame before processing.
  static const size_t BUF_SIZE = 512;  // room for at least 2 back-to-back frames
  struct ClientSlot {
    int fd{-1};
    uint8_t buf[512];
    size_t len{0};      // bytes currently in buf
    void reset() { fd = -1; len = 0; }
  };
  ClientSlot clients_[MAX_CLIENTS];
  int listen_fd_{-1};
  uint32_t last_start_attempt_{0};
};

}  // namespace em24_meter
}  // namespace esphome
