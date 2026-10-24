/*
 * Si4432 (EZRadioPRO rev B1) register definitions and the d101-2 channel profile.
 *
 * Source: Silicon Labs Si4432 datasheet (register descriptions, "RX Modem
 * Configuration" section 7.1 and Table 17 "Filter Bandwidth Parameters"). Every
 * modem value below is computed from those formulas in si4432_hal.cpp, not copied
 * from a calculator export.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::nartis_rf_2_meter::si4432 {

/* Registers */
static constexpr uint8_t REG_DEVICE_TYPE = 0x00;
static constexpr uint8_t REG_DEVICE_VERSION = 0x01;
static constexpr uint8_t REG_DEVICE_STATUS = 0x02;
static constexpr uint8_t REG_INT_STATUS1 = 0x03;
static constexpr uint8_t REG_INT_STATUS2 = 0x04;
static constexpr uint8_t REG_INT_ENABLE1 = 0x05;
static constexpr uint8_t REG_INT_ENABLE2 = 0x06;
static constexpr uint8_t REG_OP_MODE1 = 0x07;
static constexpr uint8_t REG_OP_MODE2 = 0x08;
static constexpr uint8_t REG_XTAL_LOAD = 0x09;
static constexpr uint8_t REG_GPIO0_CFG = 0x0B;
static constexpr uint8_t REG_GPIO1_CFG = 0x0C;
static constexpr uint8_t REG_GPIO2_CFG = 0x0D;
static constexpr uint8_t REG_IF_FILTER_BW = 0x1C;
static constexpr uint8_t REG_AFC_GEARSHIFT = 0x1D;
static constexpr uint8_t REG_AFC_TIMING = 0x1E;
static constexpr uint8_t REG_CR_GEARSHIFT = 0x1F;
static constexpr uint8_t REG_CR_OSR = 0x20;
static constexpr uint8_t REG_CR_OFFSET2 = 0x21;
static constexpr uint8_t REG_CR_OFFSET1 = 0x22;
static constexpr uint8_t REG_CR_OFFSET0 = 0x23;
static constexpr uint8_t REG_CR_GAIN1 = 0x24;
static constexpr uint8_t REG_CR_GAIN0 = 0x25;
static constexpr uint8_t REG_RSSI = 0x26;
static constexpr uint8_t REG_AFC_LIMITER = 0x2A;
static constexpr uint8_t REG_AFC_CORRECTION = 0x2B;
static constexpr uint8_t REG_DATA_ACCESS = 0x30;
static constexpr uint8_t REG_HEADER_CTL1 = 0x32;
static constexpr uint8_t REG_HEADER_CTL2 = 0x33;
static constexpr uint8_t REG_PREAMBLE_LEN = 0x34;
static constexpr uint8_t REG_PREAMBLE_DET = 0x35;
static constexpr uint8_t REG_SYNC3 = 0x36;  // first sync byte on air; 0x37..0x39 follow
static constexpr uint8_t REG_PKT_LEN = 0x3E;
static constexpr uint8_t REG_TX_POWER = 0x6D;
static constexpr uint8_t REG_TX_DR1 = 0x6E;
static constexpr uint8_t REG_TX_DR0 = 0x6F;
static constexpr uint8_t REG_MOD_CTL1 = 0x70;
static constexpr uint8_t REG_MOD_CTL2 = 0x71;
static constexpr uint8_t REG_FREQ_DEV = 0x72;
static constexpr uint8_t REG_FREQ_OFFSET1 = 0x73;
static constexpr uint8_t REG_FREQ_OFFSET2 = 0x74;
static constexpr uint8_t REG_BAND_SELECT = 0x75;
static constexpr uint8_t REG_CARRIER1 = 0x76;
static constexpr uint8_t REG_CARRIER0 = 0x77;
static constexpr uint8_t REG_RX_FIFO_CTL = 0x7E;
static constexpr uint8_t REG_FIFO = 0x7F;

static constexpr uint8_t SPI_WRITE = 0x80;

static constexpr uint8_t DEVICE_TYPE_TRX = 0x08;

/* 0x02 device status */
static constexpr uint8_t STATUS_FFOVFL = 0x80;
static constexpr uint8_t STATUS_FFUNFL = 0x40;
static constexpr uint8_t STATUS_RXFFEM = 0x20;
static constexpr uint8_t STATUS_FREQERR = 0x08;

/* 0x03 / 0x05 */
static constexpr uint8_t INT1_FFERR = 0x80;
static constexpr uint8_t INT1_RXFFAFULL = 0x10;
static constexpr uint8_t INT1_PKSENT = 0x04;
static constexpr uint8_t INT1_PKVALID = 0x02;
/* 0x04 / 0x06 */
static constexpr uint8_t INT2_SWDET = 0x80;
static constexpr uint8_t INT2_PREAVAL = 0x40;
static constexpr uint8_t INT2_CHIPRDY = 0x02;

/* 0x07 */
static constexpr uint8_t OP1_SWRES = 0x80;
static constexpr uint8_t OP1_TXON = 0x08;
static constexpr uint8_t OP1_RXON = 0x04;
static constexpr uint8_t OP1_XTON = 0x01;
/* 0x08 */
static constexpr uint8_t OP2_FFCLRRX = 0x02;
static constexpr uint8_t OP2_FFCLRTX = 0x01;

/* GPIO functions */
static constexpr uint8_t GPIO_TX_STATE = 0x12;
static constexpr uint8_t GPIO_RX_STATE = 0x15;

/* 0x30 */
static constexpr uint8_t DA_ENPACRX = 0x80;
static constexpr uint8_t DA_ENPACTX = 0x08;
/* 0x33: hdlen=0 (no header), fixpklen=1, synclen in bits 2:1 = bytes-1 */
static constexpr uint8_t HC2_FIXPKLEN = 0x08;
/* 0x70: txdtrtscale for data rates below 30 kbps; no Manchester, no whitening */
static constexpr uint8_t MOD1_TXDTRTSCALE = 0x20;
/* 0x71: dtmod = FIFO, modtyp = GFSK */
static constexpr uint8_t MOD2_FIFO_GFSK = 0x23;

static constexpr size_t FIFO_SIZE = 64;
/// RX almost-full threshold (0x7E): this many bytes are known to be in the FIFO
/// when rxffafull is set, so they can be read in one burst.
static constexpr uint8_t RX_AFULL_THRESHOLD = 48;
static constexpr uint32_t XTAL_HZ = 30000000u;

/* d101-2 link, from the CMT2300A profile this component was proven with:
 *   both directions 1.2 kbps (G)FSK;
 *   display -> meter: ~4 kHz deviation, preamble 0x55, then the frame LSB-first;
 *   meter -> display: ~25 kHz deviation (h ~ 41), sync 98 F3 after the preamble,
 *   arriving ~4.7 kHz above the display's carrier (758 CMT codes x 6.199 Hz).
 */
static constexpr uint32_t BIT_RATE_BPS = 1200;
static constexpr uint32_t TX_DEVIATION_HZ = 4000;
static constexpr uint32_t RX_DEVIATION_HZ_DEFAULT = 25000;
static constexpr int32_t RX_OFFSET_HZ_DEFAULT = 4699;

/* The CMT2300A sends 10 preamble bytes, a 4-byte sync blended to 0x55 and 6 more
 * 0x55 pad bytes before the frame: 20 bytes of 0x55 on air. The Si4432 packet
 * handler reproduces exactly that stream - see Si4432Hal::transmit(). */
static constexpr size_t CMT_LEAD_BYTES = 20;
static constexpr uint8_t CMT_LEAD_VALUE = 0x55;

/* RX: 3-byte sync = the meter's last preamble byte + 98 F3, all as they appear on
 * air MSB-first (the frame itself is LSB-first): 55 19 CF. Same as the CMT path. */
static constexpr uint8_t RX_SYNC[3] = {0x55, 0x19, 0xCF};
/* Fixed RX length, as the CMT path: longer than any reply (<= 146 bytes); the frame's
 * own LEN byte ends reception early. */
static constexpr uint8_t RX_FIXED_LEN = 180;

/* Table 17 entries reachable at 1.2 kbps: rxosr = 500 kHz / (2^(ndec-3) * Rb) must
 * fit 11 bits, which needs ndec_exp >= 1 (ndec 0 would be 3333). dwn3_bypass = 0. */
struct IfFilter {
  uint16_t bw_100hz;  // -3 dB bandwidth in 100 Hz units
  uint8_t ndec_exp;
  uint8_t filset;
};
static constexpr IfFilter IF_FILTERS[] = {
    {377, 1, 1}, {417, 1, 2}, {452, 1, 3}, {479, 1, 4}, {562, 1, 5}, {641, 1, 6}, {692, 1, 7},
};

/* TX power code (0x6D txpow) -> dBm, Si4432. */
static constexpr int8_t TX_POWER_DBM[8] = {1, 2, 5, 8, 11, 14, 17, 20};

}  // namespace esphome::nartis_rf_2_meter::si4432
