// Host-side check of si4432_modem.cpp:
//   - the RX modem formulas against the Si4432 datasheet's Table 16 examples
//   - the d101-2 RX profile (1.2 kbps, 25 kHz deviation)
//   - the TX bit stream: what the Si4432 puts on air must be, bit for bit, what the
//     CMT2300A path puts on air (20 x 0x55, then the frame LSB-first)
#include "d101_frame.h"
#include "si4432_defs.h"
#include "si4432_modem.h"

#include <cstdio>
#include <cstring>

using namespace esphome::nartis_rf_2_meter;
using namespace esphome::nartis_rf_2_meter::si4432;

static int failures = 0;

static void check(bool ok, const char *what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    failures++;
}

struct Row {
  uint32_t rb, fd;
  uint8_t ndec, filset;
  uint16_t rxosr;
  uint32_t ncoff;
  uint16_t crgain;
};

// Si4432 datasheet rev 0.4, Table 16, every row with dwn3_bypass = 0.
static const Row TABLE16[] = {
    {2000, 5000, 3, 3, 0x0FA, 0x08312, 0x06B},   {2400, 4800, 3, 3, 0x0D0, 0x09D49, 0x0A0},
    {2400, 36000, 0, 1, 0x683, 0x013A9, 0x005},  {4800, 4800, 3, 4, 0x068, 0x13A93, 0x278},
    {4800, 45000, 0, 4, 0x341, 0x02752, 0x00A},  {9600, 4800, 2, 1, 0x068, 0x13A93, 0x4EE},
    {9600, 45000, 0, 4, 0x1A1, 0x04EA5, 0x024},  {10000, 5000, 2, 1, 0x064, 0x147AE, 0x521},
    {10000, 40000, 0, 3, 0x190, 0x051EC, 0x02B}, {19200, 9600, 1, 1, 0x068, 0x13A93, 0x4EE},
    {20000, 10000, 1, 1, 0x064, 0x147AE, 0x521}, {20000, 40000, 0, 4, 0x0C8, 0x0A3D7, 0x0A6},
    {38400, 19600, 0, 1, 0x068, 0x13A93, 0x4D5}, {40000, 20000, 0, 1, 0x064, 0x147AE, 0x521},
    {40000, 40000, 0, 5, 0x064, 0x147AE, 0x291}, {50000, 25000, 0, 1, 0x050, 0x1999A, 0x668},
    {57600, 28800, 0, 3, 0x045, 0x1D7DC, 0x76E},
};

int main() {
  // ---- modem formulas ----
  int bad = 0;
  for (const Row &r : TABLE16) {
    const ModemRegs m = compute_modem_with(r.rb, r.fd, r.ndec, r.filset, 0);
    const bool ok = m.rxosr == r.rxosr && m.ncoff == r.ncoff && m.crgain == r.crgain &&
                    m.if_bw == ((r.ndec << 4) | r.filset);
    if (!ok) {
      bad++;
      std::printf("  %5u bps %5u Hz: got %03X %05X %03X, table %03X %05X %03X\n", (unsigned) r.rb, (unsigned) r.fd,
                  m.rxosr, (unsigned) m.ncoff, m.crgain, r.rxosr, (unsigned) r.ncoff, r.crgain);
    }
  }
  std::printf("Table 16: %d of %zu rows differ\n", bad, sizeof(TABLE16) / sizeof(TABLE16[0]));
  check(bad == 0, "rxosr / ncoff / crgain formulas reproduce datasheet Table 16");

  const ModemRegs d = compute_modem(BIT_RATE_BPS, RX_DEVIATION_HZ_DEFAULT, 64000);
  std::printf("d101-2 RX: 0x1C=%02X rxosr=%03X ncoff=%05X crgain=%03X bw=%.1f kHz\n", d.if_bw, d.rxosr,
              (unsigned) d.ncoff, d.crgain, d.bw_hz / 1000.0);
  check(d.if_bw == 0x16 && d.bw_hz == 64100, "64 kHz request picks the 64.1 kHz filter (ndec 1, filset 6)");
  check(d.rxosr == 0x683 && d.rxosr <= 0x7FF, "rxosr 0x683 fits 11 bits at 1.2 kbps");
  check(d.ncoff == 0x013A9, "ncoff 0x013A9");
  check(d.crgain == 4, "crgain 4 for h = 41.7");
  const ModemRegs a = compute_modem(BIT_RATE_BPS, RX_DEVIATION_HZ_DEFAULT, 0);
  check(a.bw_hz == 56200, "auto bandwidth: 2*(Fd + Rb/4) = 50.6 kHz -> 56.2 kHz filter");
  const ModemRegs w = compute_modem(BIT_RATE_BPS, RX_DEVIATION_HZ_DEFAULT, 500000);
  check(w.bw_hz == 69200, "an over-wide request is clamped to 69.2 kHz, the widest legal at 1.2 kbps");

  // ---- TX stream ----
  uint8_t serial[SERIAL_BCD_SIZE];
  serial_to_bcd_le("023240271060", serial);
  bool all_ok = true;
  size_t biggest = 0;
  for (const ListRequest &req : LIST_REQUESTS) {
    uint8_t frame[MAX_REQUEST_FRAME_SIZE];
    const size_t len = build_request(frame, sizeof(frame), serial, req.di);
    uint8_t sync[4], payload[FIFO_SIZE];
    const size_t n = build_tx_stream(frame, len, sync, payload, sizeof(payload));
    if (n == 0) {
      all_ok = false;
      continue;
    }
    biggest = n > biggest ? n : biggest;

    uint8_t bits[1024];
    const size_t nb = on_air_bits(sync, payload, n, bits, sizeof(bits));
    // Expected: one extra preamble bit, then D = 20 x 0x55, then the frame LSB-first.
    size_t at = 1;
    bool ok = bits[0] == 1 && sync[0] == 0xAA && sync[1] == 0xAA && sync[2] == 0xAA && sync[3] == 0xAA;
    for (size_t b = 0; b < CMT_LEAD_BYTES && ok; b++)
      for (int i = 7; i >= 0; i--)
        ok &= bits[at++] == ((CMT_LEAD_VALUE >> i) & 1);
    for (size_t b = 0; b < len && ok; b++)
      for (int i = 0; i < 8; i++)  // LSB-first on air
        ok &= bits[at++] == ((frame[b] >> i) & 1);
    ok &= at <= nb && nb - at < 8;  // only zero padding to the byte boundary remains
    for (size_t i = at; i < nb && ok; i++)
      ok &= bits[i] == 0;
    if (!ok) {
      std::printf("  DI 0x%04X: on-air stream differs from the CMT2300A stream\n", req.di);
      all_ok = false;
    }
    if (req.di == DI_LIST_2_RECORDS) {
      std::printf("DI 0xF202: %zu-byte frame -> sync %02X %02X %02X %02X + %zu FIFO bytes, %zu bits on air\n", len,
                  sync[0], sync[1], sync[2], sync[3], n, nb);
    }
  }
  check(all_ok, "Si4432 TX bit stream == CMT2300A stream (20 x 0x55 + frame LSB-first) for every list request");
  check(biggest <= FIFO_SIZE, "every request fits the 64-byte TX FIFO in one load");

  // The 3-byte RX sync, read MSB-first, is the meter's last preamble byte plus 98 F3 LSB-first.
  check(RX_SYNC[0] == 0x55 && RX_SYNC[1] == reverse8(D101_SYNC0) && RX_SYNC[2] == reverse8(D101_SYNC1),
        "RX sync 55 19 CF == on-air form of [AA] 98 F3");

  std::printf("\n%s (%d failure(s))\n", failures ? "FAILED" : "ALL PASSED", failures);
  return failures ? 1 : 0;
}
