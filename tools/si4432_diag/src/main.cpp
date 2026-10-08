/*
 * Si4432 wiring / health check for ESP32 DevKit V1.
 *
 * RECEIVE-ONLY. The chip is never put into TX: txon is never written, and the
 * only RF activity is listening for RSSI on the meter channel grid.
 *
 * Wiring (as built):
 *   Si4432  SCLK  SDI   SDO   NSEL  NIRQ  SDN
 *   ESP32   18    23    19    5     27    26
 *
 * Serial commands (115200 baud, one letter + Enter):
 *   d  rerun the full diagnostic
 *   s  RSSI sweep over all 24 meter channels
 *   l  listen on one channel: "l 443.9" (MHz), prints RSSI peaks for 30 s
 *   a  antenna switch probe (GPIO0/GPIO1 polarity) on the last listened channel
 */

#include <Arduino.h>
#include <SPI.h>

namespace {

constexpr int PIN_SCLK = 18;
constexpr int PIN_MOSI = 23;  // Si4432 SDI
constexpr int PIN_MISO = 19;  // Si4432 SDO
constexpr int PIN_NSEL = 5;
constexpr int PIN_NIRQ = 27;
constexpr int PIN_SDN = 26;

// Datasheet Si4432 rev 0.4: SPI up to 10 MHz, mode 0, MSB first, A7 = 1 for write.
uint32_t g_spi_hz = 1000000;
SPIClass g_spi(VSPI);

uint32_t g_fails = 0;
uint32_t g_warns = 0;
float g_last_mhz = 443.9f;

// Meter channel grid - same rule as components/nartis_rf_2_meter/cmt2300a_defs.h.
uint32_t channel_hz(uint32_t k) { return 435500000u + k * 700000u + (k > 18 ? 100000u : 0u); }

void ok(const char *fmt, ...) {
  char b[200];
  va_list a;
  va_start(a, fmt);
  vsnprintf(b, sizeof(b), fmt, a);
  va_end(a);
  Serial.printf("  [ OK ] %s\n", b);
}
void warn(const char *fmt, ...) {
  char b[200];
  va_list a;
  va_start(a, fmt);
  vsnprintf(b, sizeof(b), fmt, a);
  va_end(a);
  Serial.printf("  [WARN] %s\n", b);
  g_warns++;
}
void fail(const char *fmt, ...) {
  char b[200];
  va_list a;
  va_start(a, fmt);
  vsnprintf(b, sizeof(b), fmt, a);
  va_end(a);
  Serial.printf("  [FAIL] %s\n", b);
  g_fails++;
}

uint8_t rd(uint8_t reg) {
  g_spi.beginTransaction(SPISettings(g_spi_hz, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_NSEL, LOW);
  g_spi.transfer(reg & 0x7F);
  uint8_t v = g_spi.transfer(0x00);
  digitalWrite(PIN_NSEL, HIGH);
  g_spi.endTransaction();
  return v;
}

void wr(uint8_t reg, uint8_t val) {
  g_spi.beginTransaction(SPISettings(g_spi_hz, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_NSEL, LOW);
  g_spi.transfer(reg | 0x80);
  g_spi.transfer(val);
  digitalWrite(PIN_NSEL, HIGH);
  g_spi.endTransaction();
}

// The SPI peripheral owns MISO; to test the bare line we briefly hand it back to GPIO.
int miso_level_with_pull(bool pullup) {
  g_spi.end();
  pinMode(PIN_MISO, pullup ? INPUT_PULLUP : INPUT_PULLDOWN);
  delayMicroseconds(200);
  int v = digitalRead(PIN_MISO);
  g_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  return v;
}

uint8_t read_with_miso_pull(uint8_t reg, bool pullup) {
  pinMode(PIN_MISO, pullup ? INPUT_PULLUP : INPUT_PULLDOWN);  // pull stays when SPI re-attaches the pad
  g_spi.end();
  g_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  if (pullup)
    gpio_pullup_en((gpio_num_t) PIN_MISO), gpio_pulldown_dis((gpio_num_t) PIN_MISO);
  else
    gpio_pulldown_en((gpio_num_t) PIN_MISO), gpio_pullup_dis((gpio_num_t) PIN_MISO);
  uint8_t v = rd(reg);
  gpio_pullup_dis((gpio_num_t) PIN_MISO);
  gpio_pulldown_dis((gpio_num_t) PIN_MISO);
  return v;
}

void clear_irq() {
  rd(0x03);
  rd(0x04);
}

// Wait for nIRQ to go low (active-low, open... the Si4432 drives it push-pull).
bool wait_nirq_low(uint32_t timeout_ms, uint32_t *took_us) {
  uint32_t t0 = micros();
  while (micros() - t0 < timeout_ms * 1000u) {
    if (digitalRead(PIN_NIRQ) == LOW) {
      if (took_us)
        *took_us = micros() - t0;
      return true;
    }
  }
  return false;
}

void set_frequency_hz(uint32_t hz) {
  const uint8_t hbsel = hz >= 480000000u ? 1 : 0;
  const double step = 10e6 * (hbsel + 1);
  const double x = hz / step;
  const uint8_t fb = (uint8_t) ((int) x - 24);
  const uint16_t fc = (uint16_t) ((x - (int) x) * 64000.0 + 0.5);
  wr(0x75, 0x40 | (hbsel << 5) | (fb & 0x1F));
  wr(0x76, fc >> 8);
  wr(0x77, fc & 0xFF);
}

// Approximation of the datasheet's RSSI curve (~0.5 dB/LSB). Good for comparing,
// not for absolute levels.
float rssi_dbm(uint8_t raw) { return raw * 0.5f - 120.0f; }

// Minimal RX setup for RSSI only: FIFO mode, GFSK, the receive filter of the
// eventual driver (64 kHz, 1.2 kbps clock recovery). No packet is decoded here.
void rx_setup_for_rssi() {
  wr(0x07, 0x01);  // READY
  wr(0x1C, 0x16);  // dwn3_bypass=0, ndec_exp=1, filset=6 -> 64.1 kHz
  wr(0x20, 0x83);  // rxosr = 0x683 (1.2 kbps)
  wr(0x21, 0xC0);  // rxosr[10:8]=6, ncoff[19:16]=0
  wr(0x22, 0x13);  // ncoff = 0x013A9
  wr(0x23, 0xA9);
  wr(0x24, 0x00);
  wr(0x25, 0x04);  // crgain
  wr(0x1D, 0x40);  // AFC on
  wr(0x71, 0x23);  // FIFO mode, GFSK
}

void rx_on() {
  wr(0x08, 0x02);  // clear RX FIFO
  wr(0x08, 0x00);
  wr(0x07, 0x05);  // RX + XTAL
  delay(2);
}

void idle() { wr(0x07, 0x01); }

struct RssiStat {
  uint8_t min{255}, max{0};
  uint32_t sum{0}, n{0};
};

RssiStat sample_rssi(uint32_t ms) {
  RssiStat s;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    uint8_t r = rd(0x26);
    s.min = min(s.min, r);
    s.max = max(s.max, r);
    s.sum += r;
    s.n++;
    delayMicroseconds(300);
  }
  return s;
}

void set_antenna_gpio(bool gpio0_is_tx) {
  // 0x12 = TX state, 0x15 = RX state (Si4432 GPIO function table).
  wr(0x0B, gpio0_is_tx ? 0x12 : 0x15);
  wr(0x0C, gpio0_is_tx ? 0x15 : 0x12);
}

/* ------------------------------------------------------------------ */

void diag() {
  g_fails = g_warns = 0;
  Serial.println();
  Serial.println("================ Si4432 diagnostic (receive-only) ================");
  Serial.printf("Pins: SCLK=%d SDI(MOSI)=%d SDO(MISO)=%d NSEL=%d NIRQ=%d SDN=%d\n", PIN_SCLK, PIN_MOSI, PIN_MISO,
                PIN_NSEL, PIN_NIRQ, PIN_SDN);

  // ---- 1. chip held in shutdown ----
  Serial.println("\n[1] SDN high (chip in shutdown)");
  g_spi_hz = 1000000;
  digitalWrite(PIN_SDN, HIGH);
  delay(20);
  uint8_t off_pu = read_with_miso_pull(0x00, true);
  uint8_t off_pd = read_with_miso_pull(0x00, false);
  Serial.printf("  reg 0x00 in shutdown: 0x%02X (MISO pull-up) / 0x%02X (pull-down)\n", off_pu, off_pd);
  if (off_pu == 0x08 && off_pd == 0x08)
    fail("chip answers while SDN is high -> SDN not connected to GPIO%d or stuck low", PIN_SDN);
  else
    ok("chip silent in shutdown (SDN works or chip unpowered - see step 2)");

  // ---- 2. power up ----
  Serial.println("\n[2] SDN low -> power-on reset");
  int nirq_before = digitalRead(PIN_NIRQ);
  digitalWrite(PIN_SDN, LOW);
  uint32_t por_us = 0;
  bool por_irq = wait_nirq_low(50, &por_us);
  delay(20);  // datasheet: POR + crystal start-up up to ~16.8 ms
  Serial.printf("  nIRQ before release: %s\n", nirq_before ? "HIGH" : "LOW");
  if (por_irq)
    ok("nIRQ went LOW %.1f ms after SDN release (POR/chip-ready interrupt) -> NIRQ wire OK", por_us / 1000.0f);
  else
    fail("nIRQ never went LOW after power-up -> check NIRQ wire to GPIO%d", PIN_NIRQ);

  // ---- 3. ID ----
  Serial.println("\n[3] Identification");
  uint8_t dt = rd(0x00), vc = rd(0x01), st = rd(0x02);
  Serial.printf("  0x00 device type = 0x%02X (expect 0x08)\n", dt);
  Serial.printf("  0x01 version     = 0x%02X (0x06 = rev B1, 0x02 = V2, 0x03/0x04 = A0)\n", vc);
  Serial.printf("  0x02 status      = 0x%02X (cps=%u)\n", st, st & 3);
  if (dt == 0x08) {
    ok("Si443x transceiver detected");
  } else if (dt == 0x00 || dt == 0xFF) {
    int pu = miso_level_with_pull(true), pd = miso_level_with_pull(false);
    fail("read 0x%02X: MISO idle with pull-up=%d pull-down=%d", dt, pu, pd);
    if (pu == 1 && pd == 0)
      Serial.println("         MISO floats -> SDO not connected to GPIO19, or chip unpowered (VDD/GND), or NSEL wrong");
    else if (pu == 0 && pd == 0)
      Serial.println("         MISO stuck LOW -> short to GND or chip held in reset");
    else if (pu == 1 && pd == 1)
      Serial.println("         MISO stuck HIGH -> short to 3V3, or SDO swapped with SDI");
    Serial.println("         Also check: SDN must be LOW to run; NSEL on GPIO5; SCLK on GPIO18; common GND.");
  } else {
    fail("unexpected device type 0x%02X -> not a Si4432, or SPI bit errors (SDI/SDO swapped? bad GND?)", dt);
  }
  if (vc != 0x06 && dt == 0x08)
    warn("version 0x%02X is not rev B1 - driver is written against B1 register defaults", vc);

  // ---- 4. interrupt status clears nIRQ ----
  Serial.println("\n[4] Interrupt status read clears nIRQ");
  uint8_t is1 = rd(0x03), is2 = rd(0x04);
  delayMicroseconds(50);
  int nirq_after = digitalRead(PIN_NIRQ);
  Serial.printf("  0x03=0x%02X 0x04=0x%02X, nIRQ now %s\n", is1, is2, nirq_after ? "HIGH" : "LOW");
  if (nirq_after == HIGH)
    ok("nIRQ released after status read");
  else
    fail("nIRQ still LOW after reading status -> NIRQ line shorted to GND?");

  // ---- 5. software reset ----
  Serial.println("\n[5] Software reset");
  clear_irq();
  wr(0x07, 0x80);
  uint32_t sw_us = 0;
  bool sw_irq = wait_nirq_low(50, &sw_us);
  delay(5);
  is1 = rd(0x03);
  is2 = rd(0x04);
  if (sw_irq && (is2 & 0x02))
    ok("chip ready %.2f ms after SWRES (0x04=0x%02X) -> SDI/MOSI writes reach the chip", sw_us / 1000.0f, is2);
  else
    fail("no chip-ready after SWRES (irq=%d, 0x04=0x%02X) -> SDI (MOSI) wire? writes not reaching chip", sw_irq, is2);

  // ---- 6. reset defaults ----
  Serial.println("\n[6] Register reset values (datasheet Table 30)");
  struct Def {
    uint8_t reg, val;
    const char *name;
  };
  const Def defs[] = {
      {0x05, 0x00, "int enable 1"}, {0x06, 0x03, "int enable 2"}, {0x07, 0x01, "op mode 1"},
      {0x1C, 0x01, "IF filter BW"}, {0x20, 0x64, "rxosr"},        {0x30, 0x8D, "data access"},
      {0x33, 0x22, "header ctl 2"}, {0x34, 0x08, "preamble len"}, {0x36, 0x2D, "sync 3"},
      {0x37, 0xD4, "sync 2"},       {0x6D, 0x18, "TX power"},     {0x6E, 0x0A, "txdr 1"},
      {0x6F, 0x3D, "txdr 0"},       {0x70, 0x0C, "mod ctl 1"},    {0x75, 0x75, "band select"},
      {0x76, 0xBB, "carrier 1"},    {0x77, 0x80, "carrier 0"},
  };
  int def_mismatch = 0;
  for (const Def &d : defs) {
    uint8_t v = rd(d.reg);
    bool same = v == d.val;
    if (!same)
      def_mismatch++;
    Serial.printf("  0x%02X %-14s = 0x%02X %s (doc 0x%02X)\n", d.reg, d.name, v, same ? "  " : "<>", d.val);
  }
  // Datasheet revisions disagree on a few defaults, so this is advisory only.
  if (def_mismatch == 0)
    ok("all reset values match");
  else
    warn("%d value(s) differ from the rev 0.4 table (revisions differ; SPI is judged by steps 7-8)", def_mismatch);

  // ---- 7. write/read-back ----
  Serial.println("\n[7] Write / read-back (sync word 3, restored afterwards)");
  uint8_t saved = rd(0x36);
  const uint8_t pats[] = {0x00, 0xFF, 0x55, 0xAA, 0x5A, 0xA5, 0x01, 0x80};
  int wr_err = 0;
  for (uint8_t p : pats) {
    wr(0x36, p);
    uint8_t r = rd(0x36);
    if (r != p) {
      wr_err++;
      Serial.printf("  wrote 0x%02X read 0x%02X\n", p, r);
    }
  }
  wr(0x36, saved);
  if (wr_err == 0)
    ok("8/8 patterns read back");
  else
    fail("%d/8 patterns wrong -> bit errors on SDI/SDO/SCLK (long wires? bad GND?)", wr_err);

  // ---- 8. SPI speed margin ----
  Serial.println("\n[8] SPI clock margin (2000 ID reads + 200 R/W per speed)");
  const uint32_t speeds[] = {1000000, 4000000, 8000000, 10000000};
  uint32_t best = 0;
  for (uint32_t hz : speeds) {
    g_spi_hz = hz;
    int err = 0;
    for (int i = 0; i < 2000; i++)
      if (rd(0x00) != 0x08)
        err++;
    for (int i = 0; i < 200; i++) {
      uint8_t p = (uint8_t) esp_random();
      wr(0x36, p);
      if (rd(0x36) != p)
        err++;
    }
    Serial.printf("  %5.1f MHz: %d errors\n", hz / 1e6, err);
    if (err == 0)
      best = hz;
  }
  wr(0x36, saved);
  g_spi_hz = 1000000;
  if (best >= 4000000)
    ok("SPI clean up to %.0f MHz (driver uses 4 MHz)", best / 1e6);
  else if (best > 0)
    warn("SPI only clean at %.0f MHz - shorten wires / improve GND", best / 1e6);
  else
    fail("SPI errors at every speed");

  // ---- 9. crystal / temperature ----
  Serial.println("\n[9] Crystal and on-chip temperature sensor");
  wr(0x07, 0x01);
  delay(5);
  st = rd(0x02);
  Serial.printf("  0x02 status = 0x%02X, 0x09 xtal load = 0x%02X\n", st, rd(0x09));
  wr(0x12, 0x20);  // temp sensor range -64..64 C, 0.5 C/LSB
  wr(0x0F, 0x80);  // ADC start, input = temperature sensor
  uint32_t t0 = millis();
  while (!(rd(0x0F) & 0x80) && millis() - t0 < 20) {
  }
  uint8_t traw = rd(0x11);
  float tc = traw * 0.5f - 64.0f;
  Serial.printf("  temperature raw 0x%02X -> %.1f C\n", traw, tc);
  if (tc > -20 && tc < 80)
    ok("ADC/temperature plausible -> chip core running on its 30 MHz crystal");
  else
    warn("temperature implausible (%.1f C)", tc);

  // ---- 10. PLL lock in RX on a meter channel ----
  Serial.println("\n[10] PLL lock (RX on 443.900 MHz, nothing transmitted)");
  rx_setup_for_rssi();
  set_antenna_gpio(true);
  set_frequency_hz(443900000);
  wr(0x07, 0x05);
  delay(5);
  st = rd(0x02);
  RssiStat s = sample_rssi(300);
  idle();
  Serial.printf("  status 0x%02X (freqerr=%u), RSSI raw min/avg/max %u/%u/%u (~%.0f/%.0f/%.0f dBm)\n", st,
                (st >> 3) & 1, s.min, (unsigned) (s.sum / s.n), s.max, rssi_dbm(s.min), rssi_dbm(s.sum / s.n),
                rssi_dbm(s.max));
  if (st & 0x08)
    fail("frequency error flag set - synthesizer out of range");
  else
    ok("synthesizer accepted 443.9 MHz");
  if (s.max == 0)
    warn("RSSI stuck at 0 - receiver chain not running?");

  Serial.println("\n------------------------------------------------------------------");
  Serial.printf("RESULT: %u fail, %u warn -> %s\n", g_fails, g_warns,
                g_fails ? "WIRING/CHIP PROBLEM, see [FAIL] lines" : "Si4432 link OK");
  Serial.println("Commands: d = rerun, s = channel sweep, l <MHz> = listen 30 s, a = antenna switch probe");
}

void sweep() {
  Serial.println("\nRSSI sweep over the 24 meter channels, 400 ms each (receive-only)");
  Serial.println("  k  freq MHz    min  avg  max (raw; ~dBm = raw/2-120)");
  rx_setup_for_rssi();
  set_antenna_gpio(true);
  for (uint32_t k = 0; k < 24; k++) {
    set_frequency_hz(channel_hz(k));
    rx_on();
    RssiStat s = sample_rssi(400);
    idle();
    Serial.printf("  %2u %8.3f   %4u %4u %4u %s\n", k, channel_hz(k) / 1e6, s.min, (unsigned) (s.sum / s.n), s.max,
                  s.max > s.sum / s.n + 30 ? "<- burst seen" : "");
  }
}

void listen(float mhz) {
  g_last_mhz = mhz;
  Serial.printf("\nListening on %.4f MHz for 30 s - press a button on the Д101-2 now. Peaks > avg+20 printed.\n",
                mhz);
  rx_setup_for_rssi();
  set_antenna_gpio(true);
  set_frequency_hz((uint32_t) (mhz * 1e6 + 0.5));
  rx_on();
  RssiStat base = sample_rssi(300);
  uint32_t floor_ = base.sum / base.n;
  Serial.printf("  noise floor raw %u (~%.0f dBm)\n", floor_, rssi_dbm(floor_));
  uint32_t t0 = millis(), last = 0;
  int bursts = 0;
  while (millis() - t0 < 30000) {
    uint8_t r = rd(0x26);
    if (r > floor_ + 20 && millis() - last > 100) {
      int8_t afc = (int8_t) rd(0x2B);
      Serial.printf("  t=%6.2f s RSSI raw %u (~%.0f dBm), AFC %+.2f kHz\n", (millis() - t0) / 1000.0f, r,
                    rssi_dbm(r), afc * 0.15625f);
      last = millis();
      bursts++;
    }
    delayMicroseconds(200);
  }
  idle();
  Serial.printf("  %d peak(s) seen\n", bursts);
}

void antenna_probe() {
  Serial.printf("\nAntenna switch probe on %.4f MHz: average RSSI with GPIO0/GPIO1 = TX/RX and swapped\n",
                g_last_mhz);
  rx_setup_for_rssi();
  set_frequency_hz((uint32_t) (g_last_mhz * 1e6 + 0.5));
  for (int pass = 0; pass < 2; pass++) {
    for (int pol = 0; pol < 2; pol++) {
      set_antenna_gpio(pol == 0);
      rx_on();
      RssiStat s = sample_rssi(1500);
      idle();
      Serial.printf("  pass %d, GPIO0=%s GPIO1=%s: avg %u max %u\n", pass + 1, pol == 0 ? "TXst" : "RXst",
                    pol == 0 ? "RXst" : "TXst", (unsigned) (s.sum / s.n), s.max);
    }
  }
  set_antenna_gpio(true);
  Serial.println("  A clearly higher average in one setting means the module has an RF switch on those pins.");
  Serial.println("  Equal values: no switch, or no signal to compare against (try with the Д101-2 active).");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_SDN, OUTPUT);
  digitalWrite(PIN_SDN, HIGH);  // start in shutdown
  pinMode(PIN_NSEL, OUTPUT);
  digitalWrite(PIN_NSEL, HIGH);
  pinMode(PIN_NIRQ, INPUT_PULLUP);
  g_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  diag();
}

void loop() {
  if (!Serial.available())
    return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.isEmpty())
    return;
  switch (line[0]) {
    case 'd':
      diag();
      break;
    case 's':
      sweep();
      break;
    case 'l': {
      float mhz = line.length() > 1 ? line.substring(1).toFloat() : g_last_mhz;
      if (mhz < 430 || mhz > 460) {
        Serial.println("frequency must be 430..460 MHz");
        break;
      }
      listen(mhz);
      break;
    }
    case 'a':
      antenna_probe();
      break;
    default:
      Serial.println("Commands: d, s, l <MHz>, a");
  }
}
