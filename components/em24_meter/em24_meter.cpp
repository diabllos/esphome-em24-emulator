#include "em24_meter.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <lwip/sockets.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <cmath>

namespace esphome {
namespace em24_meter {

static const char *const TAG = "em24_meter";

// raw = round(value * scale), returned as 32-bit signed
static inline int32_t scaled_(float value, float scale) {
  if (std::isnan(value))
    return 0;
  return static_cast<int32_t>(lroundf(value * scale));
}
// low word of a 32-bit value (Reg_s32l => low word at lower address)
static inline uint16_t lo16_(int32_t v) { return static_cast<uint16_t>(static_cast<uint32_t>(v) & 0xFFFF); }
static inline uint16_t hi16_(int32_t v) { return static_cast<uint16_t>((static_cast<uint32_t>(v) >> 16) & 0xFFFF); }

void EM24Meter::setup() {
  for (int i = 0; i < MAX_CLIENTS; i++)
    this->clients_[i].reset();
  this->start_server_();
}

bool EM24Meter::start_server_() {
  this->listen_fd_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (this->listen_fd_ < 0) {
    ESP_LOGW(TAG, "socket() failed: errno %d", errno);
    return false;
  }

  int opt = 1;
  setsockopt(this->listen_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(this->port_);

  if (bind(this->listen_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ESP_LOGW(TAG, "bind() failed on port %u: errno %d (network up yet?)", this->port_, errno);
    ::close(this->listen_fd_);
    this->listen_fd_ = -1;
    return false;
  }
  if (listen(this->listen_fd_, 2) < 0) {
    ESP_LOGW(TAG, "listen() failed: errno %d", errno);
    ::close(this->listen_fd_);
    this->listen_fd_ = -1;
    return false;
  }
  fcntl(this->listen_fd_, F_SETFL, O_NONBLOCK);
  ESP_LOGI(TAG, "EM24 Modbus TCP server listening on port %u (unit id %u)", this->port_, this->unit_id_);
  return true;
}

void EM24Meter::loop() {
  // (Re)start the server if it isn't up yet (e.g. network came up after setup()).
  if (this->listen_fd_ < 0) {
    uint32_t now = millis();
    if (now - this->last_start_attempt_ > 5000) {
      this->last_start_attempt_ = now;
      this->start_server_();
    }
    return;
  }

  // Accept a pending connection (non-blocking).
  int fd = accept(this->listen_fd_, nullptr, nullptr);
  if (fd >= 0) {
    fcntl(fd, F_SETFL, O_NONBLOCK);
    int slot = -1;
    for (int i = 0; i < MAX_CLIENTS; i++) {
      if (this->clients_[i].fd < 0) {
        slot = i;
        break;
      }
    }
    if (slot < 0) {
      ::close(fd);  // no free slot
    } else {
      this->clients_[slot].fd = fd;
      this->clients_[slot].len = 0;
      ESP_LOGD(TAG, "client connected (slot %d)", slot);
    }
  }

  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (this->clients_[i].fd >= 0)
      this->service_client_(i);
  }
}

void EM24Meter::service_client_(int slot) {
  ClientSlot &c = this->clients_[slot];

  // Read whatever is available into the tail of the buffer.
  size_t room = BUF_SIZE - c.len;
  if (room == 0) {
    // Buffer full without a valid frame — client is sending garbage.  Drop it.
    ESP_LOGW(TAG, "client buffer overflow (slot %d), disconnecting", slot);
    ::close(c.fd);
    c.reset();
    return;
  }

  int n = recv(c.fd, c.buf + c.len, room, 0);
  if (n == 0) {  // peer closed
    ::close(c.fd);
    c.reset();
    ESP_LOGD(TAG, "client disconnected (slot %d)", slot);
    return;
  }
  if (n < 0) {
    if (errno == EWOULDBLOCK || errno == EAGAIN)
      return;  // nothing to read right now
    ::close(c.fd);
    c.reset();
    return;
  }
  c.len += static_cast<size_t>(n);

  // Process as many complete Modbus TCP frames as the buffer contains.
  // MBAP header: [0-1] tid, [2-3] protocol, [4-5] length (big-endian).
  // Total frame size = 6 (header) + length.
  size_t pos = 0;
  while (pos + 6 <= c.len) {
    uint16_t mbap_len = (static_cast<uint16_t>(c.buf[pos + 4]) << 8) | c.buf[pos + 5];
    // Sanity: Modbus TCP length field should be 1..253 (unit id + max PDU).
    if (mbap_len == 0 || mbap_len > 253) {
      ESP_LOGW(TAG, "bad MBAP length %u (slot %d), disconnecting", mbap_len, slot);
      ::close(c.fd);
      c.reset();
      return;
    }
    size_t frame_size = 6 + mbap_len;
    if (pos + frame_size > c.len)
      break;  // incomplete frame — wait for more data

    uint8_t resp[260];
    size_t resp_len = this->process_frame_(c.buf + pos, frame_size, resp);
    if (resp_len > 0) {
      // Send full response, handling partial writes.
      size_t sent = 0;
      while (sent < resp_len) {
        int w = send(c.fd, resp + sent, resp_len - sent, 0);
        if (w < 0) {
          if (errno == EWOULDBLOCK || errno == EAGAIN)
            continue;  // retry (rare on small payloads)
          ::close(c.fd);
          c.reset();
          return;
        }
        sent += static_cast<size_t>(w);
      }
    }
    pos += frame_size;
  }

  // Compact: move any leftover partial frame to the front of the buffer.
  if (pos > 0) {
    c.len -= pos;
    if (c.len > 0)
      memmove(c.buf, c.buf + pos, c.len);
  }
}

// Modbus TCP: 7-byte MBAP header (tid, pid, len, unit) + PDU.
size_t EM24Meter::process_frame_(const uint8_t *req, size_t len, uint8_t *resp) {
  if (len < 8)
    return 0;
  uint16_t pid = (req[2] << 8) | req[3];
  if (pid != 0)
    return 0;
  uint8_t uid = req[6];
  uint8_t fc = req[7];

  // copy transaction id + protocol id, set unit id; length is filled per-case
  resp[0] = req[0];
  resp[1] = req[1];
  resp[2] = 0;
  resp[3] = 0;
  resp[6] = uid;

  auto set_len = [&](uint16_t pdu_bytes) {
    uint16_t l = pdu_bytes + 1;  // + unit id
    resp[4] = (l >> 8) & 0xFF;
    resp[5] = l & 0xFF;
  };

  // Read Holding (0x03) / Read Input (0x04) — answered identically.
  if (fc == 0x03 || fc == 0x04) {
    if (len < 12)
      return 0;
    uint16_t start = (req[8] << 8) | req[9];
    uint16_t qty = (req[10] << 8) | req[11];
    if (qty < 1 || qty > 125) {
      set_len(2);
      resp[7] = fc | 0x80;
      resp[8] = 0x03;  // illegal data value
      return 9;
    }
    uint8_t byte_count = qty * 2;
    set_len(2 + byte_count);
    resp[7] = fc;
    resp[8] = byte_count;
    for (uint16_t i = 0; i < qty; i++) {
      uint16_t v = this->get_register_(start + i);
      resp[9 + i * 2] = (v >> 8) & 0xFF;
      resp[9 + i * 2 + 1] = v & 0xFF;
    }
    return 9 + byte_count;
  }

  // Write Single Register (0x06) — accept config writes from the GX, echo back.
  if (fc == 0x06) {
    if (len < 12)
      return 0;
    uint16_t addr = (req[8] << 8) | req[9];
    uint16_t val = (req[10] << 8) | req[11];
    if (addr == 0xA000)
      this->app_reg_ = val;
    if (addr == 0x1002)
      this->phase_config_ = val;
    set_len(5);
    for (int i = 7; i < 12; i++)
      resp[i] = req[i];
    return 12;
  }

  // Write Multiple Registers (0x10) — accept and echo address/quantity.
  if (fc == 0x10) {
    if (len < 13)
      return 0;
    set_len(5);
    resp[7] = req[7];
    resp[8] = req[8];
    resp[9] = req[9];
    resp[10] = req[10];
    resp[11] = req[11];
    return 12;
  }

  // Unsupported function -> exception "illegal function".
  set_len(2);
  resp[7] = fc | 0x80;
  resp[8] = 0x01;
  return 9;
}

uint16_t EM24Meter::get_register_(uint16_t addr) {
  float p_L1 = (this->power_L1 && this->power_L1->has_state()) ? this->power_L1->state : 0.0f;
  float p_L2 = (this->power_L2 && this->power_L2->has_state()) ? this->power_L2->state : 0.0f;
  float p_L3 = (this->power_L3 && this->power_L3->has_state()) ? this->power_L3->state : 0.0f;
  if (this->invert_power_){
    p_L1 = -p_L1;
    p_L2 = -p_L2;
    p_L3 = -p_L3;
  }
  float p = p_L1 + p_L2 + p_L3;
  float imp = (this->import_ && this->import_->has_state()) ? this->import_->state : 0.0f;
  float exp = (this->export_ && this->export_->has_state()) ? this->export_->state : 0.0f;
  float volt_L1 = (this->voltage_L1 && this->voltage_L1->has_state()) ? this->voltage_L1->state : 230.0f;
  float volt_L2 = (this->voltage_L2 && this->voltage_L2->has_state()) ? this->voltage_L2->state : 230.0f;
  float volt_L3 = (this->voltage_L3 && this->voltage_L3->has_state()) ? this->voltage_L3->state : 230.0f;
  float freq = (this->frequency_ && this->frequency_->has_state()) ? this->frequency_->state : 50.0f;
  if (std::isnan(volt_L1) || volt_L1 < 1.0f)
    volt_L1 = 230.0f;
  if (std::isnan(volt_L2) || volt_L2 < 1.0f)
    volt_L2 = 230.0f;
  if (std::isnan(volt_L3) || volt_L3 < 1.0f)
    volt_L3 = 230.0f;

  if (std::isnan(freq))
    freq = 50.0f;
  // synthesize a plausible current from power when no CT sensor is provided
  float curr_L1 = std::isnan(p_L1) ? 0.0f : (fabsf(p_L1) / volt_L1);
  float curr_L2 = std::isnan(p_L2) ? 0.0f : (fabsf(p_L2) / volt_L2);
  float curr_L3 = std::isnan(p_L3) ? 0.0f : (fabsf(p_L3) / volt_L3);

  int32_t power_total_raw = scaled_(p, 10.0f);
  int32_t imp_raw = scaled_(imp, 10.0f);
  int32_t exp_raw = scaled_(exp, 10.0f);
  int32_t volt_L1_raw = scaled_(volt_L1, 10.0f);
  int32_t volt_L2_raw = scaled_(volt_L2, 10.0f);
  int32_t volt_L3_raw = scaled_(volt_L3, 10.0f);
  int32_t curr_L1_raw = scaled_(curr_L1, 1000.0f);
  int32_t curr_L2_raw = scaled_(curr_L2, 1000.0f);
  int32_t curr_L3_raw = scaled_(curr_L3, 1000.0f);
  int32_t power_L1_raw = scaled_(p_L1, 1000.0f);
  int32_t power_L2_raw = scaled_(p_L2, 1000.0f);
  int32_t power_L3_raw = scaled_(p_L3, 1000.0f);



  switch (addr) {
    // --- probe: model id (Victron accepts 1648..1653) ---
    case 0x000B: return 1651;  // EM24DINAV53XE1X

    // --- L1 phase block ---
    case 0x0000: return lo16_(volt_L1_raw);   // V L1
    case 0x0001: return hi16_(volt_L1_raw);
    case 0x0002: return lo16_(volt_L2_raw);   // V L2
    case 0x0003: return hi16_(volt_L2_raw);
    case 0x0004: return lo16_(volt_L3_raw);   // V L3
    case 0x0005: return hi16_(volt_L3_raw);

    case 0x000C: return lo16_(curr_L1_raw);   // A L1
    case 0x000D: return hi16_(curr_L1_raw);
    case 0x000E: return lo16_(curr_L2_raw);   // A L2
    case 0x000F: return hi16_(curr_L2_raw);
    case 0x0010: return lo16_(curr_L3_raw);   // A L3
    case 0x0011: return hi16_(curr_L3_raw);

    case 0x0012: return lo16_(power_L1_raw);  // W  L1
    case 0x0013: return hi16_(power_L1_raw);
    case 0x0014: return lo16_(power_L2_raw);  // W  L2
    case 0x0015: return hi16_(power_L2_raw);
    case 0x0016: return lo16_(power_L3_raw);  // W  L3
    case 0x0017: return hi16_(power_L3_raw);

    case 0x0040: return lo16_(imp_raw);    // kWh forward L1
    case 0x0041: return hi16_(imp_raw);

    // --- system / totals ---
    case 0x0028: return lo16_(power_total_raw);  // W system  (/Ac/Power)
    case 0x0029: return hi16_(power_total_raw);

    case 0x0032: return 0;                 // phase sequence (3-phase only)
    case 0x0033: return static_cast<uint16_t>(scaled_(freq, 10.0f));  // Hz
    case 0x0034: return lo16_(imp_raw);    // kWh forward total
    case 0x0035: return hi16_(imp_raw);
    case 0x004E: return lo16_(exp_raw);    // kWh reverse total
    case 0x004F: return hi16_(exp_raw);

    // --- info / config ---
    case 0x0302: return 0x0101;            // HardwareVersion -> "0.1.1"
    case 0x0304: return 0x0101;            // FirmwareVersion -> "0.1.1"
    case 0x1002: return this->phase_config_;  // PhaseConfig (3 = 1P)
    case 0xA000: return this->app_reg_;       // application, must read 7 ("H")
    case 0xA100: return 0;                    // switch position (0 = not "Locked")

    default: break;
  }

  // Serial string: 7 registers @0x5000, big-endian char pairs.
  if (addr >= 0x5000 && addr <= 0x5006) {
    int i = (addr - 0x5000) * 2;
    uint8_t a = (i < static_cast<int>(this->serial_.size())) ? static_cast<uint8_t>(this->serial_[i]) : 0;
    uint8_t b = (i + 1 < static_cast<int>(this->serial_.size())) ? static_cast<uint8_t>(this->serial_[i + 1]) : 0;
    return (static_cast<uint16_t>(a) << 8) | b;
  }

  return 0;
}

void EM24Meter::dump_config() {
  ESP_LOGCONFIG(TAG, "EM24 Modbus TCP emulator:");
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
  ESP_LOGCONFIG(TAG, "  Unit id: %u", this->unit_id_);
  ESP_LOGCONFIG(TAG, "  PhaseConfig reg: %u", this->phase_config_);
  ESP_LOGCONFIG(TAG, "  Serial: %s", this->serial_.c_str());
  ESP_LOGCONFIG(TAG, "  Invert power: %s", this->invert_power_ ? "true" : "false");
}

}  // namespace em24_meter
}  // namespace esphome
