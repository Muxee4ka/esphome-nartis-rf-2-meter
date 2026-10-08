/*
 * Si4432 pure helpers - no ESPHome dependencies, so tools/frame_check can test them
 * on the host.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::nartis_rf_2_meter::si4432 {

/// RX modem registers for FSK/GFSK, Manchester off (datasheet section 7.1.1).
struct ModemRegs {
  uint8_t if_bw;    // 0x1C
  uint16_t rxosr;   // 0x20, 0x21[7:5]
  uint32_t ncoff;   // 0x21[3:0], 0x22, 0x23
  uint16_t crgain;  // 0x24[2:0], 0x25
  uint32_t bw_hz;   // the Table 17 filter actually chosen
};

/// `want_bw_hz` = 0 derives the filter from BW = 2 * (Fd + Rb / 4).
/// Only the dwn3_bypass = 0, ndec_exp = 1 filters are offered: at 1.2 kbps they are
/// the widest whose rxosr fits 11 bits.
ModemRegs compute_modem(uint32_t bit_rate_bps, uint32_t deviation_hz, uint32_t want_bw_hz);

/// Same, for any Table 17 row with dwn3_bypass = 0 - used by the host test to check
/// the formulas against the datasheet's Table 16 examples.
ModemRegs compute_modem_with(uint32_t bit_rate_bps, uint32_t deviation_hz, uint8_t ndec_exp, uint8_t filset,
                             uint32_t bw_hz);

/// Preamble the packet handler sends ahead of the sync word, in nibbles.
static constexpr uint8_t TX_PREAMBLE_NIBBLES = 32;

/// Split the CMT2300A on-air stream for `frame` into what the Si4432 packet handler
/// needs: a 4-byte sync word and the FIFO payload. Returns the payload length, 0 if
/// it does not fit `payload_cap` or the 64-byte FIFO.
size_t build_tx_stream(const uint8_t *frame, size_t len, uint8_t sync[4], uint8_t *payload, size_t payload_cap);

/// Reproduce the bits the Si4432 puts on air for (preamble, sync, payload), MSB
/// first, as a byte string - the inverse view used by the host test.
size_t on_air_bits(const uint8_t sync[4], const uint8_t *payload, size_t n, uint8_t *bits, size_t cap);

uint8_t reverse8(uint8_t b);

}  // namespace esphome::nartis_rf_2_meter::si4432
