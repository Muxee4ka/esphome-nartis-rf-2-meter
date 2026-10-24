/*
 * Si4432 HAL - d101-2 link. See si4432_hal.h for the scheme.
 */

#include "si4432_hal.h"

#ifdef USE_SPI

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cmath>
#include <cstring>


namespace esphome::nartis_rf_2_meter {

using namespace si4432;

static const char *const TAG = "si4432_hal";

static constexpr uint32_t TX_TIMEOUT_MS = 1200;  // ~49 bytes at 1.2 kbps = 330 ms
static constexpr uint32_t READY_TIMEOUT_MS = 50;
/// The receiver is said to be measured in ~0.5 dB steps from ~-120 dBm (datasheet
/// RSSI vs input power curve). Good for trends, not calibrated.
static int8_t rssi_raw_to_dbm(uint8_t raw) { return static_cast<int8_t>(static_cast<int>(raw) / 2 - 120); }

// ---------------------------------------------------------------- SPI

uint8_t Si4432Hal::read_reg_(uint8_t reg) {
  this->enable();
  this->write_byte(reg & 0x7F);
  uint8_t v = this->read_byte();
  this->disable();
  return v;
}

void Si4432Hal::write_reg_(uint8_t reg, uint8_t val) {
  this->enable();
  this->write_byte(reg | SPI_WRITE);
  this->write_byte(val);
  this->disable();
}

void Si4432Hal::read_burst_(uint8_t reg, uint8_t *data, size_t len) {
  this->enable();
  this->write_byte(reg & 0x7F);
  this->read_array(data, len);
  this->disable();
}

void Si4432Hal::write_burst_(uint8_t reg, const uint8_t *data, size_t len) {
  this->enable();
  this->write_byte(reg | SPI_WRITE);
  this->write_array(data, len);
  this->disable();
}

// ---------------------------------------------------------------- setup

void Si4432Hal::set_carrier_(int64_t hz) {
  // Datasheet 3.5.1: low band (hbsel = 0) for 240-480 MHz, 10 MHz per fb step,
  // fc in 1/64000 of that (156.25 Hz).
  const uint8_t hbsel = hz >= 480000000 ? 1 : 0;
  const int64_t step = 10000000LL * (hbsel + 1);
  const int64_t fb = hz / step - 24;
  const int64_t rem = hz - (fb + 24) * step;
  int64_t fc = (rem * 64000 + step / 2) / step;
  if (fc > 0xFFFF)
    fc = 0xFFFF;
  this->write_reg_(REG_BAND_SELECT, static_cast<uint8_t>(0x40 | (hbsel << 5) | (fb & 0x1F)));
  this->write_reg_(REG_CARRIER1, static_cast<uint8_t>(fc >> 8));
  this->write_reg_(REG_CARRIER0, static_cast<uint8_t>(fc & 0xFF));
}

void Si4432Hal::clear_irq_() {
  this->read_reg_(REG_INT_STATUS1);
  this->read_reg_(REG_INT_STATUS2);
}

bool Si4432Hal::wait_ready_(uint32_t timeout_ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (this->read_reg_(REG_INT_STATUS2) & INT2_CHIPRDY)
      return true;
    delayMicroseconds(200);
  }
  return false;
}

bool Si4432Hal::init() {
  this->spi_setup();
  if (this->nirq_pin_ != nullptr) {
    this->nirq_pin_->setup();
  }
  if (this->sdn_pin_ != nullptr) {
    // Power-cycle through shutdown: the only full reset the chip has.
    this->sdn_pin_->setup();
    this->sdn_pin_->digital_write(true);
    delay(10);
    this->sdn_pin_->digital_write(false);
    delay(25);  // POR + crystal start-up, datasheet <= 16.8 ms
  }

  const uint8_t dt = this->read_reg_(REG_DEVICE_TYPE);
  this->version_ = this->read_reg_(REG_DEVICE_VERSION);
  if (dt != DEVICE_TYPE_TRX) {
    ESP_LOGE(TAG, "Si4432 not detected: device type 0x%02X (expected 0x08). 0x00/0xFF = SDO not wired, chip "
                  "unpowered or held in shutdown",
             dt);
    return false;
  }
  ESP_LOGD(TAG, "Si4432 device type 0x%02X, version 0x%02X", dt, this->version_);

  this->clear_irq_();
  this->write_reg_(REG_OP_MODE1, OP1_SWRES);
  if (!this->wait_ready_(READY_TIMEOUT_MS)) {
    ESP_LOGE(TAG, "Si4432 did not report chip-ready after software reset");
    return false;
  }
  this->clear_irq_();
  this->write_reg_(REG_INT_ENABLE1, 0x00);
  this->write_reg_(REG_INT_ENABLE2, 0x00);
  this->write_reg_(REG_OP_MODE1, OP1_XTON);  // READY

  if (this->xtal_load_ >= 0) {
    this->write_reg_(REG_XTAL_LOAD, static_cast<uint8_t>(this->xtal_load_));
  }

  switch (this->antenna_switch_) {
    case Si4432AntennaSwitch::ANTENNA_SWITCH_GPIO0_TX:
      this->write_reg_(REG_GPIO0_CFG, GPIO_TX_STATE);
      this->write_reg_(REG_GPIO1_CFG, GPIO_RX_STATE);
      break;
    case Si4432AntennaSwitch::ANTENNA_SWITCH_GPIO0_RX:
      this->write_reg_(REG_GPIO0_CFG, GPIO_RX_STATE);
      this->write_reg_(REG_GPIO1_CFG, GPIO_TX_STATE);
      break;
    case Si4432AntennaSwitch::ANTENNA_SWITCH_NONE:
      break;
  }

  // Modulation, both directions: FIFO mode, GFSK, 1.2 kbps, no Manchester/whitening.
  const uint32_t txdr = static_cast<uint32_t>(((static_cast<uint64_t>(BIT_RATE_BPS) << 21) + 500000u) / 1000000u);
  this->write_reg_(REG_TX_DR1, static_cast<uint8_t>(txdr >> 8));
  this->write_reg_(REG_TX_DR0, static_cast<uint8_t>(txdr & 0xFF));
  this->write_reg_(REG_MOD_CTL1, MOD1_TXDTRTSCALE);
  this->write_reg_(REG_MOD_CTL2, MOD2_FIFO_GFSK);  // fd[8] = 0
  this->write_reg_(REG_FREQ_DEV, static_cast<uint8_t>((TX_DEVIATION_HZ + 312) / 625));
  this->write_reg_(REG_FREQ_OFFSET1, 0);
  this->write_reg_(REG_FREQ_OFFSET2, 0);
  this->write_reg_(REG_TX_POWER, static_cast<uint8_t>(0x08 | this->tx_power_code_));  // lna_sw | txpow

  // RX modem, sized for the meter's wide-deviation reply.
  this->modem_ = compute_modem(BIT_RATE_BPS, this->rx_dev_hz_, this->rx_bw_hz_);
  this->write_reg_(REG_IF_FILTER_BW, this->modem_.if_bw);
  this->write_reg_(REG_AFC_GEARSHIFT, 0x40);  // enafc
  this->write_reg_(REG_AFC_TIMING, 0x08);     // shwait = 1, lgwait = 0 (AFC frozen after preamble)
  this->write_reg_(REG_CR_GEARSHIFT, 0x05);   // crfast = 0, crslow = 5 (datasheet recommendation)
  this->write_reg_(REG_CR_OSR, static_cast<uint8_t>(this->modem_.rxosr & 0xFF));
  this->write_reg_(REG_CR_OFFSET2,
                   static_cast<uint8_t>(((this->modem_.rxosr >> 3) & 0xE0) | ((this->modem_.ncoff >> 16) & 0x0F)));
  this->write_reg_(REG_CR_OFFSET1, static_cast<uint8_t>((this->modem_.ncoff >> 8) & 0xFF));
  this->write_reg_(REG_CR_OFFSET0, static_cast<uint8_t>(this->modem_.ncoff & 0xFF));
  this->write_reg_(REG_CR_GAIN1, static_cast<uint8_t>((this->modem_.crgain >> 8) & 0x07));
  this->write_reg_(REG_CR_GAIN0, static_cast<uint8_t>(this->modem_.crgain & 0xFF));
  this->write_reg_(REG_AFC_LIMITER, 24);  // +-15 kHz pull-in (x 625 Hz)

  // Packet handler: no header, fixed length, no CRC (the frame has its own).
  this->write_reg_(REG_HEADER_CTL1, 0x00);
  this->write_reg_(REG_PREAMBLE_DET, 5 << 3);  // 20-bit preamble detect, as advised with AFC on
  this->write_reg_(REG_RX_FIFO_CTL, RX_AFULL_THRESHOLD);

  this->set_carrier_(static_cast<int64_t>(this->rf_freq_hz_) + this->freq_corr_hz_);
  this->write_reg_(REG_OP_MODE1, OP1_XTON);
  delay(2);
  if (this->read_reg_(REG_DEVICE_STATUS) & STATUS_FREQERR) {
    ESP_LOGE(TAG, "Si4432 rejected %" PRIu32 " Hz (frequency error flag)", this->rf_freq_hz_);
    return false;
  }
  ESP_LOGI(TAG, "Si4432 rev 0x%02X initialized at %.4f MHz", this->version_, this->rf_freq_hz_ / 1e6);
  return true;
}

void Si4432Hal::dump_config() {
  ESP_LOGCONFIG(TAG, "  Radio: Si4432 (version 0x%02X), hardware SPI 4 MHz", this->version_);
  LOG_PIN("    nIRQ pin: ", this->nirq_pin_);
  LOG_PIN("    SDN pin: ", this->sdn_pin_);
  ESP_LOGCONFIG(TAG, "    TX power: %+d dBm, deviation %" PRIu32 " Hz", TX_POWER_DBM[this->tx_power_code_],
                (uint32_t) ((TX_DEVIATION_HZ + 312) / 625 * 625));
  ESP_LOGCONFIG(TAG, "    Frequency correction: %+" PRId32 " Hz, RX offset %+" PRId32 " Hz", this->freq_corr_hz_,
                this->rx_offset_hz_);
  ESP_LOGCONFIG(TAG, "    RX: deviation %" PRIu32 " Hz (h=%.1f), IF filter %.1f kHz (0x1C=0x%02X), rxosr 0x%03X, "
                     "ncoff 0x%05" PRIX32 ", crgain 0x%03X",
                this->rx_dev_hz_, 2.0f * this->rx_dev_hz_ / BIT_RATE_BPS, this->modem_.bw_hz / 1000.0f,
                this->modem_.if_bw, this->modem_.rxosr, this->modem_.ncoff, this->modem_.crgain);
  if (2 * this->rx_dev_hz_ > 32 * BIT_RATE_BPS) {
    ESP_LOGCONFIG(TAG, "    NOTE: h > 32 is outside the datasheet's specified demodulator range");
  }
  ESP_LOGCONFIG(TAG, "    FIFO over/underflows: %" PRIu32 ", RX almost-full burst at %u bytes", this->fifo_errors_,
                RX_AFULL_THRESHOLD);
}

// ---------------------------------------------------------------- TX

bool Si4432Hal::transmit(const uint8_t *frame, size_t len) {
  uint8_t sync[4];
  uint8_t payload[FIFO_SIZE];
  const size_t n = build_tx_stream(frame, len, sync, payload, sizeof(payload));
  if (n == 0) {
    ESP_LOGW(TAG, "TX frame too big for the 64-byte FIFO (%zu)", len);
    return false;
  }

  this->write_reg_(REG_OP_MODE1, OP1_XTON);
  this->set_carrier_(static_cast<int64_t>(this->rf_freq_hz_) + this->freq_corr_hz_);
  this->write_reg_(REG_DATA_ACCESS, DA_ENPACTX);
  this->write_reg_(REG_HEADER_CTL2, HC2_FIXPKLEN | (3 << 1));  // 4-byte sync, prealen[8] = 0
  this->write_reg_(REG_PREAMBLE_LEN, TX_PREAMBLE_NIBBLES);
  this->write_burst_(REG_SYNC3, sync, 4);
  this->write_reg_(REG_PKT_LEN, static_cast<uint8_t>(n));
  this->write_reg_(REG_OP_MODE2, OP2_FFCLRTX | OP2_FFCLRRX);
  this->write_reg_(REG_OP_MODE2, 0x00);
  this->write_burst_(REG_FIFO, payload, n);

  this->write_reg_(REG_INT_ENABLE1, INT1_PKSENT);
  this->write_reg_(REG_INT_ENABLE2, 0x00);
  this->clear_irq_();
  this->write_reg_(REG_OP_MODE1, OP1_TXON | OP1_XTON);

  const uint32_t t0 = millis();
  while (millis() - t0 < TX_TIMEOUT_MS) {
    const bool irq = (this->nirq_pin_ == nullptr) || !this->nirq_pin_->digital_read();
    if (irq && (this->read_reg_(REG_INT_STATUS1) & INT1_PKSENT)) {
      ESP_LOGV(TAG, "TX done: %zu-byte frame as %zu FIFO bytes in %" PRIu32 " ms", len, n, millis() - t0);
      this->write_reg_(REG_OP_MODE1, OP1_XTON);
      return true;
    }
    delay(1);
  }
  ESP_LOGW(TAG, "TX: packet-sent not seen within %" PRIu32 " ms (status 0x%02X)", TX_TIMEOUT_MS,
           this->read_reg_(REG_DEVICE_STATUS));
  this->write_reg_(REG_OP_MODE1, OP1_XTON);
  return false;
}

// ---------------------------------------------------------------- RX

bool Si4432Hal::begin_rx(int /*cmt_off_codes*/) {
  this->write_reg_(REG_OP_MODE1, OP1_XTON);
  this->set_carrier_(static_cast<int64_t>(this->rf_freq_hz_) + this->freq_corr_hz_ + this->rx_offset_hz_);
  this->write_reg_(REG_DATA_ACCESS, DA_ENPACRX);
  this->write_reg_(REG_HEADER_CTL2, HC2_FIXPKLEN | (2 << 1));  // 3-byte sync
  this->write_burst_(REG_SYNC3, RX_SYNC, sizeof(RX_SYNC));
  this->write_reg_(REG_PKT_LEN, RX_FIXED_LEN);
  this->write_reg_(REG_OP_MODE2, OP2_FFCLRRX);
  this->write_reg_(REG_OP_MODE2, 0x00);
  this->write_reg_(REG_INT_ENABLE1, INT1_FFERR | INT1_RXFFAFULL | INT1_PKVALID);
  this->write_reg_(REG_INT_ENABLE2, INT2_SWDET);
  this->clear_irq_();
  this->sync_seen_ = false;
  this->sync_rssi_dbm_ = -128;
  this->last_drain_ms_ = 0;
  this->write_reg_(REG_OP_MODE1, OP1_RXON | OP1_XTON);
  return true;
}

uint8_t Si4432Hal::service_irq_() {
  if (this->nirq_pin_ != nullptr && this->nirq_pin_->digital_read()) {
    return 0;  // nothing pending
  }
  const uint8_t s1 = this->read_reg_(REG_INT_STATUS1);
  const uint8_t s2 = this->read_reg_(REG_INT_STATUS2);
  if ((s2 & INT2_SWDET) && !this->sync_seen_) {
    // AFC froze at preamble detect and RSSI still reflects the signal: read both now.
    this->sync_seen_ = true;
    this->sync_rssi_dbm_ = rssi_raw_to_dbm(this->read_reg_(REG_RSSI));
    this->sync_afc_raw_ = static_cast<int8_t>(this->read_reg_(REG_AFC_CORRECTION));
    ESP_LOGV(TAG, "sync word: RSSI %d dBm, AFC %+.2f kHz", this->sync_rssi_dbm_, this->sync_afc_raw_ * 0.15625f);
  }
  if (s1 & INT1_FFERR) {
    this->fifo_errors_++;
    const uint8_t st = this->read_reg_(REG_DEVICE_STATUS);
    const uint32_t gap = this->last_drain_ms_ != 0 ? millis() - this->last_drain_ms_ : 0;
    ESP_LOGW(TAG, "RX FIFO error (status 0x%02X:%s%s), %" PRIu32 " ms since the previous drain", st,
             (st & STATUS_FFOVFL) ? " overflow" : "", (st & STATUS_FFUNFL) ? " underflow" : "", gap);
  }
  return s1;
}

size_t Si4432Hal::drain_rx(uint8_t *buf, size_t buf_size, uint8_t /*max_chunks*/) {
  const uint32_t now = millis();
  if (this->sync_seen_ && this->last_drain_ms_ != 0 && now - this->last_drain_ms_ > this->max_drain_gap_ms_) {
    this->max_drain_gap_ms_ = now - this->last_drain_ms_;
  }
  this->last_drain_ms_ = now;

  const uint8_t s1 = this->service_irq_();
  size_t n = 0;
  if ((s1 & INT1_RXFFAFULL) && buf_size >= RX_AFULL_THRESHOLD) {
    // At least the threshold is waiting: one burst instead of 2 x 48 transactions.
    this->read_burst_(REG_FIFO, buf, RX_AFULL_THRESHOLD);
    for (; n < RX_AFULL_THRESHOLD; n++)
      buf[n] = reverse8(buf[n]);
  }
  // No byte counter on this chip: pop while "RX FIFO empty" is clear.
  while (n < buf_size && !(this->read_reg_(REG_DEVICE_STATUS) & STATUS_RXFFEM)) {
    buf[n++] = reverse8(this->read_reg_(REG_FIFO));
  }
  if (n > 0 && !this->sync_seen_) {
    // nIRQ not wired, or the status was read elsewhere: fall back to a live sample.
    this->sync_seen_ = true;
    this->sync_rssi_dbm_ = rssi_raw_to_dbm(this->read_reg_(REG_RSSI));
    this->sync_afc_raw_ = static_cast<int8_t>(this->read_reg_(REG_AFC_CORRECTION));
  }
  return n;
}

int8_t Si4432Hal::get_rssi_dbm() {
  if (this->sync_seen_) {
    ESP_LOGD(TAG, "reply AFC correction %+.2f kHz, longest gap between FIFO reads %" PRIu32 " ms",
             this->sync_afc_raw_ * 0.15625f, this->max_drain_gap_ms_);
    this->max_drain_gap_ms_ = 0;
    return this->sync_rssi_dbm_;
  }
  return rssi_raw_to_dbm(this->read_reg_(REG_RSSI));
}

bool Si4432Hal::go_standby() {
  this->write_reg_(REG_OP_MODE1, OP1_XTON);
  this->write_reg_(REG_INT_ENABLE1, 0x00);
  this->write_reg_(REG_INT_ENABLE2, 0x00);
  this->clear_irq_();
  return true;
}

}  // namespace esphome::nartis_rf_2_meter

#endif  // USE_SPI
