/*
 * Project 1 frame definitions, shared by the F429 and H723 firmware so both
 * boards agree on IDs and payload layout.
 *
 * Payload (8 bytes, little-endian):
 *   bytes 0-3  counter: up by 1 per frame, so the receiver can spot lost frames
 *   bytes 4-5  temperature of the sender's chip, in 0.01 degC (signed)
 *   bytes 6-7  sender's VDDA (analog supply voltage), in mV
 */
#ifndef PROJECT1_FRAMES_H
#define PROJECT1_FRAMES_H

#include <stdint.h>

#define P1_ID_F429   0x100U   /* frames sent by the F429 */
#define P1_ID_H723   0x200U   /* frames sent by the H723 */
#define P1_DLC       8U

/* Plausible ranges for received sensor values. A frame outside them is
 * counted as bad: the sender's ADC or conversion is broken, or the layout
 * doesn't match. */
#define P1_TEMP_MIN_C100   (-4000)   /* -40 degC */
#define P1_TEMP_MAX_C100   12500     /* 125 degC */
#define P1_VDDA_MIN_MV     2700U
#define P1_VDDA_MAX_MV     3600U

typedef struct {
    uint32_t count;
    int16_t  temp_c100;
    uint16_t vdda_mv;
} p1_payload_t;

static inline void p1_pack(uint8_t data[8], const p1_payload_t *p)
{
    uint16_t t = (uint16_t)p->temp_c100;
    data[0] = (uint8_t)(p->count);
    data[1] = (uint8_t)(p->count >> 8);
    data[2] = (uint8_t)(p->count >> 16);
    data[3] = (uint8_t)(p->count >> 24);
    data[4] = (uint8_t)(t);
    data[5] = (uint8_t)(t >> 8);
    data[6] = (uint8_t)(p->vdda_mv);
    data[7] = (uint8_t)(p->vdda_mv >> 8);
}

static inline p1_payload_t p1_unpack(const uint8_t data[8])
{
    p1_payload_t p;
    p.count     = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                  ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    p.temp_c100 = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
    p.vdda_mv   = (uint16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));
    return p;
}

static inline int p1_payload_plausible(const p1_payload_t *p)
{
    return p->temp_c100 >= P1_TEMP_MIN_C100 && p->temp_c100 <= P1_TEMP_MAX_C100 &&
           p->vdda_mv >= P1_VDDA_MIN_MV && p->vdda_mv <= P1_VDDA_MAX_MV;
}

/*
 * Factory calibration of the internal temperature sensor and VREFINT (the
 * chip's internal reference voltage, about 1.2 V), read from system memory.
 * Both chips store these as 12-bit ADC readings taken at VDDA = 3.3 V.
 */
typedef struct {
    uint16_t vrefint_cal;   /* VREFINT reading at VDDA = cal_vdda_mv */
    uint16_t ts_cal1;       /* temperature sensor reading at ts_cal1_c */
    uint16_t ts_cal2;       /* temperature sensor reading at ts_cal2_c */
    int32_t  ts_cal1_c;
    int32_t  ts_cal2_c;
    uint32_t cal_vdda_mv;   /* 3300 on both chips */
} p1_sensor_cal_t;

/*
 * Convert 12-bit ADC readings of VREFINT and the temperature sensor:
 *
 *   VDDA = cal_vdda * VREFINT_CAL / vrefint_raw
 *          (VREFINT is fixed, so a lower reading means a higher VDDA)
 *   ts_at_cal_vdda = ts_raw * VDDA / cal_vdda
 *          (rescale the reading to what it would be at the calibration VDDA)
 *   T = cal1_c + (ts_at_cal_vdda - TS_CAL1) * (cal2_c - cal1_c) / (TS_CAL2 - TS_CAL1)
 *          (straight line through the two calibration points)
 *
 * Written out rather than using the HAL's __LL_ADC_CALC_TEMPERATURE: on the
 * H723 that macro scales the reading to 16 bits but this chip's calibration
 * values are 12-bit.
 */
static inline void p1_sensor_convert(const p1_sensor_cal_t *c, uint32_t vrefint_raw,
                                     uint32_t ts_raw, int16_t *temp_c100, uint16_t *vdda_mv)
{
    if (vrefint_raw == 0) {
        *temp_c100 = 0;
        *vdda_mv = 0;
        return;
    }
    uint32_t vdda = c->cal_vdda_mv * c->vrefint_cal / vrefint_raw;
    int64_t ts_scaled = (int64_t)ts_raw * vdda;                  /* = ts_at_cal_vdda * cal_vdda */
    int64_t num = (ts_scaled - (int64_t)c->ts_cal1 * c->cal_vdda_mv) *
                  (c->ts_cal2_c - c->ts_cal1_c) * 100;
    int64_t den = (int64_t)(c->ts_cal2 - c->ts_cal1) * c->cal_vdda_mv;
    *temp_c100 = (int16_t)(c->ts_cal1_c * 100 + num / den);
    *vdda_mv   = (uint16_t)vdda;
}

/*
 * Receive-side bookkeeping, identical on both boards. Readable over SWD.
 *   ok      frames with the right ID and DLC and plausible sensor values
 *   bad     frames with a wrong ID or DLC, or implausible sensor values
 *   lost    gaps in the counter (frames the sender sent that never arrived)
 *   restart the sender's counter went back to 1 (it was reset or reflashed)
 */
typedef struct {
    uint32_t ok;
    uint32_t bad;
    uint32_t lost;
    uint32_t restart;
    uint32_t last_count;    /* counter of the last good frame, 0 = none yet */
    uint32_t last_ok_tick;  /* HAL tick of the last good frame */
    int32_t  last_temp_c100; /* sensor values from the last good frame */
    uint32_t last_vdda_mv;
} p1_rx_stats_t;

/* Count a received frame whose ID, DLC and sensor values are already known good */
static inline void p1_rx_count(volatile p1_rx_stats_t *s, const p1_payload_t *p, uint32_t tick)
{
    uint32_t count = p->count;

    if (s->last_count != 0 && count == 1) {
        s->restart++;
    } else if (s->last_count != 0 && count > s->last_count + 1) {
        s->lost += count - s->last_count - 1;
    }
    s->last_count     = count;
    s->last_ok_tick   = tick;
    s->last_temp_c100 = p->temp_c100;
    s->last_vdda_mv   = p->vdda_mv;
    s->ok++;
}

/*
 * Bus errors by type, counted from the CAN controller's last error code
 * (LEC). bxCAN and FDCAN use the same codes:
 *   1 stuff      6 equal bits in a row where the sender must insert a stuff bit
 *   2 form       a fixed-format bit (CRC delimiter, ACK delimiter, EOF) was wrong
 *   3 ACK        sent a frame and no node acknowledged it
 *   4 bit 1      sent recessive, read back dominant (outside arbitration)
 *   5 bit 0      sent dominant, read back recessive
 *   6 CRC        received CRC didn't match the frame
 * Index 0 and 7 are unused.
 */
typedef struct {
    uint32_t lec[8];
} p1_err_counts_t;

#endif /* PROJECT1_FRAMES_H */
