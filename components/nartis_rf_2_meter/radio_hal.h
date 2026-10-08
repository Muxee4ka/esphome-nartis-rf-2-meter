/*
 * Radio seam between the polling state machine and a transceiver driver.
 *
 * The protocol layer (d101_frame.*) and the state machine only ever hand the radio
 * a complete d101-2 frame (98 F3 .. CRC) to send, and take back the bytes that
 * follow the 98 F3 sync of a reply, LEN first. Everything chip-specific - bit
 * order on air, preamble/sync handling, FIFO access, RSSI - lives behind this
 * interface. Implementations: Cmt2300aHal (the original), Si4432Hal.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::nartis_rf_2_meter {

class RadioHal {
 public:
  virtual ~RadioHal() = default;

  /// Short chip name for the log.
  virtual const char *name() const = 0;

  /// RF channel in Hz. Call before init(), which is what programs it.
  virtual void set_frequency(uint32_t freq_hz) = 0;

  /// Bring the chip up and load the static configuration. False = not detected.
  virtual bool init() = 0;

  /// Send one d101-2 frame (98 F3 .. CRC), blocking until it is on air. Leaves the
  /// chip idle.
  virtual bool transmit(const uint8_t *frame, size_t len) = 0;

  /// Enter RX for the meter's reply. `cmt_off_codes` is the CMT2300A-specific
  /// rx_center_offset; other chips carry their own offset and ignore it.
  virtual bool begin_rx(int cmt_off_codes) = 0;

  /// Append received bytes (the ones after 98 F3, LEN first). Returns bytes added.
  virtual size_t drain_rx(uint8_t *buf, size_t buf_size, uint8_t max_chunks = 5) = 0;

  /// Signal strength of the frame being / just received, dBm.
  virtual int8_t get_rssi_dbm() = 0;

  virtual bool go_standby() = 0;

  /// Chip-specific lines for dump_config().
  virtual void dump_config() {}
};

}  // namespace esphome::nartis_rf_2_meter
