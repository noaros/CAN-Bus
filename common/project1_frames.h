/*
 * Project 1 frame definitions, shared by the F429 and H723 firmware so both
 * boards agree on IDs and payload layout.
 *
 * Payload (8 bytes): counter in bytes 0-3 (little-endian), then the fixed
 * pattern CA FE BA BE in bytes 4-7. The counter goes up by 1 per frame, so
 * the receiver can spot lost frames; the pattern catches corrupted ones.
 */
#ifndef PROJECT1_FRAMES_H
#define PROJECT1_FRAMES_H

#include <stdint.h>

#define P1_ID_F429   0x100U   /* frames sent by the F429 */
#define P1_ID_H723   0x200U   /* frames sent by the H723 */

/* Decoys for testing the acceptance filters (bring-up step 7): each board
 * also sends a frame whose ID is one bit off its real one. The other
 * board's filter must drop it in hardware, so it never reaches software. */
#define P1_ID_F429_DECOY  0x101U
#define P1_ID_H723_DECOY  0x201U
#define P1_DLC       8U

static inline void p1_fill_payload(uint8_t data[8], uint32_t count)
{
    data[0] = (uint8_t)(count);
    data[1] = (uint8_t)(count >> 8);
    data[2] = (uint8_t)(count >> 16);
    data[3] = (uint8_t)(count >> 24);
    data[4] = 0xCA;
    data[5] = 0xFE;
    data[6] = 0xBA;
    data[7] = 0xBE;
}

static inline uint32_t p1_payload_count(const uint8_t data[8])
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static inline int p1_payload_pattern_ok(const uint8_t data[8])
{
    return data[4] == 0xCA && data[5] == 0xFE && data[6] == 0xBA && data[7] == 0xBE;
}

/*
 * Receive-side bookkeeping, identical on both boards. Readable over SWD.
 *   ok      frames with the right ID, DLC and pattern
 *   bad     frames with a wrong ID, DLC or pattern
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
} p1_rx_stats_t;

/* Count a received frame whose ID, DLC and pattern are already known good */
static inline void p1_rx_count(volatile p1_rx_stats_t *s, uint32_t count, uint32_t tick)
{
    if (s->last_count != 0 && count == 1) {
        s->restart++;
    } else if (s->last_count != 0 && count > s->last_count + 1) {
        s->lost += count - s->last_count - 1;
    }
    s->last_count   = count;
    s->last_ok_tick = tick;
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
