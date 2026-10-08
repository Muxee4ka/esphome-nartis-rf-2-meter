/*
 * Si4432 driver for the d101-2 link (hardware SPI via the ESPHome `spi:` bus).
 *
 * The CMT2300A path (cmt2300a_hal.*) is the reference this reproduces:
 *
 *   TX  The display's request goes out as 20 bytes of 0x55 followed by the frame,
 *       LSB-first, 1.2 kbps GFSK, ~4 kHz deviation. The Si4432 packet handler
 *       always prepends its own preamble, which starts with a 1 (1010...), where
 *       the CMT stream starts with a 0 (0x55 = 0101...). Prefixing the CMT stream
 *       with a single 1 bit makes the two identical, so the Si4432 preamble and a
 *       4-byte sync of 0xAA carry the first 160 bits and the FIFO carries the rest,
 *       bit-shifted by one. The meter sees exactly the bit sequence the CMT sends.
 *
 *   RX  Preamble detect, then 3-byte sync 55 19 CF (on-air form of [AA] 98 F3),
 *       fixed-length capture. The Si4432 has no RX byte counter, so the FIFO is
 *       drained one byte at a time while the "RX FIFO empty" status bit is clear.
 *       Bytes are bit-reversed in software (lsbfrst is not used: the datasheet
 *       does not say whether it also applies to the sync word).
 *
 *   nIRQ  Polled from loop(): when low, the interrupt status is read, which also
 *       latches RSSI and the AFC correction at sync-word detection - the only
 *       moment both describe the meter's signal rather than noise.
 *
 * Known limit: the datasheet specifies the FSK demodulator for modulation index
 * 0.5..32 (section 7.1.1). The meter's reply is h ~= 41 per the CMT profile, so
 * reception is outside the documented range and must be proven on hardware.
 */

#pragma once

#include "esphome/core/defines.h"
#ifdef USE_SPI

#include "esphome/components/spi/spi.h"
#include "esphome/core/gpio.h"

#include "radio_hal.h"
#include "si4432_defs.h"
#include "si4432_modem.h"

namespace esphome::nartis_rf_2_meter {

enum class Si4432AntennaSwitch : uint8_t {
  ANTENNA_SWITCH_NONE,          // leave GPIO0/1 alone
  ANTENNA_SWITCH_GPIO0_TX,      // GPIO0 = TX state, GPIO1 = RX state (RFM22 / most XL4432 boards)
  ANTENNA_SWITCH_GPIO0_RX,      // swapped
};

class Si4432Hal : public RadioHal,
                  public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                        spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_4MHZ> {
 public:
  void set_nirq_pin(InternalGPIOPin *pin) { this->nirq_pin_ = pin; }
  void set_sdn_pin(GPIOPin *pin) { this->sdn_pin_ = pin; }
  void set_tx_power_code(uint8_t code) { this->tx_power_code_ = code & 0x07; }
  void set_frequency_correction_hz(int32_t hz) { this->freq_corr_hz_ = hz; }
  void set_rx_offset_hz(int32_t hz) { this->rx_offset_hz_ = hz; }
  void set_rx_deviation_hz(uint32_t hz) { this->rx_dev_hz_ = hz; }
  void set_rx_bandwidth_hz(uint32_t hz) { this->rx_bw_hz_ = hz; }
  void set_antenna_switch(Si4432AntennaSwitch s) { this->antenna_switch_ = s; }
  void set_xtal_load(int16_t v) { this->xtal_load_ = v; }

  const char *name() const override { return "Si4432"; }
  void set_frequency(uint32_t freq_hz) override { this->rf_freq_hz_ = freq_hz; }
  bool init() override;
  bool transmit(const uint8_t *frame, size_t len) override;
  bool begin_rx(int cmt_off_codes) override;
  size_t drain_rx(uint8_t *buf, size_t buf_size, uint8_t max_chunks = 5) override;
  int8_t get_rssi_dbm() override;
  bool go_standby() override;
  void dump_config() override;

 protected:
  uint8_t read_reg_(uint8_t reg);
  void write_reg_(uint8_t reg, uint8_t val);
  void write_burst_(uint8_t reg, const uint8_t *data, size_t len);
  void set_carrier_(int64_t hz);
  void clear_irq_();
  void service_irq_();
  bool wait_ready_(uint32_t timeout_ms);

  InternalGPIOPin *nirq_pin_{nullptr};
  GPIOPin *sdn_pin_{nullptr};

  uint32_t rf_freq_hz_{443900000};
  int32_t freq_corr_hz_{0};
  int32_t rx_offset_hz_{si4432::RX_OFFSET_HZ_DEFAULT};
  uint32_t rx_dev_hz_{si4432::RX_DEVIATION_HZ_DEFAULT};
  uint32_t rx_bw_hz_{0};  // 0 = derive from deviation
  uint8_t tx_power_code_{3};  // +8 dBm: the Д101-2 is specified at up to 10 mW
  int16_t xtal_load_{-1};     // -1 = chip default
  Si4432AntennaSwitch antenna_switch_{Si4432AntennaSwitch::ANTENNA_SWITCH_GPIO0_TX};
  si4432::ModemRegs modem_{};
  uint8_t version_{0};

  // Latched at sync-word detection.
  bool sync_seen_{false};
  int8_t sync_rssi_dbm_{-128};
  int8_t sync_afc_raw_{0};
  uint32_t fifo_errors_{0};
};

}  // namespace esphome::nartis_rf_2_meter

#endif  // USE_SPI
