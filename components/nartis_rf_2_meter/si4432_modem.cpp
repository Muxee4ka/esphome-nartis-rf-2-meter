#include "si4432_modem.h"

#include "si4432_defs.h"

#include <cmath>

namespace esphome::nartis_rf_2_meter::si4432 {

uint8_t reverse8(uint8_t b) {
  b = static_cast<uint8_t>((b & 0xF0) >> 4 | (b & 0x0F) << 4);
  b = static_cast<uint8_t>((b & 0xCC) >> 2 | (b & 0x33) << 2);
  b = static_cast<uint8_t>((b & 0xAA) >> 1 | (b & 0x55) << 1);
  return b;
}

/* Datasheet section 7.1 / 7.1.1 and the OOK section's formulas, Manchester off,
 * dwn3_bypass = 0:
 *   rxosr  = 500 kHz / (2^(ndec-3) * Rb)
 *   ncoff  = Rb * 2^(20 + ndec) / 500 kHz
 *   crgain = 2 + 2^16 * Rb / (rxosr * Fd)
 */
ModemRegs compute_modem_with(uint32_t bit_rate_bps, uint32_t deviation_hz, uint8_t ndec_exp, uint8_t filset,
                             uint32_t bw_hz) {
  const double rb_k = bit_rate_bps / 1000.0;
  const double fd_k = deviation_hz / 1000.0;
  ModemRegs m{};
  m.bw_hz = bw_hz;
  m.if_bw = static_cast<uint8_t>(((ndec_exp & 7) << 4) | (filset & 0x0F));
  const long osr = std::lround(500.0 / (std::ldexp(1.0, ndec_exp - 3) * rb_k));
  m.rxosr = static_cast<uint16_t>(osr > 0x7FF ? 0x7FF : osr);
  m.ncoff = static_cast<uint32_t>(std::lround(rb_k * std::ldexp(1.0, 20 + ndec_exp) / 500.0)) & 0xFFFFFu;
  const long cr = std::lround(2.0 + 65536.0 * rb_k / (m.rxosr * fd_k));
  m.crgain = static_cast<uint16_t>(cr > 0x7FF ? 0x7FF : cr);
  return m;
}

ModemRegs compute_modem(uint32_t bit_rate_bps, uint32_t deviation_hz, uint32_t want_bw_hz) {
  if (want_bw_hz == 0) {
    want_bw_hz = static_cast<uint32_t>(2.0 * (deviation_hz + 0.25 * bit_rate_bps));
  }
  const size_t count = sizeof(IF_FILTERS) / sizeof(IF_FILTERS[0]);
  const IfFilter *pick = &IF_FILTERS[count - 1];  // widest when nothing is wide enough
  for (size_t i = 0; i < count; i++) {
    if (IF_FILTERS[i].bw_100hz * 100u >= want_bw_hz) {
      pick = &IF_FILTERS[i];
      break;
    }
  }
  return compute_modem_with(bit_rate_bps, deviation_hz, pick->ndec_exp, pick->filset, pick->bw_100hz * 100u);
}

/* The CMT2300A puts on air: CMT_LEAD_BYTES x 0x55, then the frame bit-reversed
 * (LSB-first). Call that bit string D; it starts 0101... The Si4432 preamble is
 * always 1010... (datasheet 6.2: "alternating ones and zeroes, starting with a
 * one"). A leading '1' turns D into D' = 1010..., so:
 *   D'[0, 128)    packet-handler preamble (32 nibbles)
 *   D'[128, 160)  sync word - works out to AA AA AA AA
 *   D'[160, ...)  FIFO payload: the last lead bit, the frame, zero padding
 * and what the meter hears is D behind one extra preamble bit.
 */
size_t build_tx_stream(const uint8_t *frame, size_t len, uint8_t sync[4], uint8_t *payload, size_t payload_cap) {
  static_assert(CMT_LEAD_BYTES * 8 + 1 >= TX_PREAMBLE_NIBBLES * 4u + 32u, "lead must cover preamble + sync");
  const size_t lead_bits = CMT_LEAD_BYTES * 8;
  const size_t total_bits = 1 + lead_bits + len * 8;
  auto bit_at = [&](size_t i) -> uint8_t {  // D'[i]
    if (i == 0) {
      return 1;
    }
    const size_t j = i - 1;  // D[j]
    if (j < lead_bits) {
      return (CMT_LEAD_VALUE >> (7 - (j & 7))) & 1;
    }
    const size_t k = j - lead_bits;
    return (reverse8(frame[k >> 3]) >> (7 - (k & 7))) & 1;
  };
  const size_t pre_bits = TX_PREAMBLE_NIBBLES * 4u;
  for (size_t b = 0; b < 4; b++) {
    uint8_t v = 0;
    for (size_t i = 0; i < 8; i++) {
      v = static_cast<uint8_t>((v << 1) | bit_at(pre_bits + b * 8 + i));
    }
    sync[b] = v;
  }
  const size_t start = pre_bits + 32;
  const size_t n = (total_bits - start + 7) / 8;
  if (n > payload_cap || n > FIFO_SIZE) {
    return 0;
  }
  for (size_t b = 0; b < n; b++) {
    uint8_t v = 0;
    for (size_t i = 0; i < 8; i++) {
      const size_t idx = start + b * 8 + i;
      v = static_cast<uint8_t>((v << 1) | (idx < total_bits ? bit_at(idx) : 0));
    }
    payload[b] = v;
  }
  return n;
}

size_t on_air_bits(const uint8_t sync[4], const uint8_t *payload, size_t n, uint8_t *bits, size_t cap) {
  size_t at = 0;
  auto put = [&](uint8_t bit) {
    if (at < cap) {
      bits[at] = bit;
    }
    at++;
  };
  for (size_t i = 0; i < TX_PREAMBLE_NIBBLES * 4u; i++) {
    put(static_cast<uint8_t>((i & 1) == 0));  // 1010...
  }
  for (size_t b = 0; b < 4; b++) {
    for (int i = 7; i >= 0; i--) {
      put((sync[b] >> i) & 1);
    }
  }
  for (size_t b = 0; b < n; b++) {
    for (int i = 7; i >= 0; i--) {
      put((payload[b] >> i) & 1);
    }
  }
  return at;
}

}  // namespace esphome::nartis_rf_2_meter::si4432
