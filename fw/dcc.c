/**
 * @file dcc.c
 * @author lowdrant
 * @brief Implements the interface described in dcc.h.
 *
 * Supports `-DPYTHON_TESTING` define flag for unit testing against Python.
 *
 * @version 0.1
 * @date 2026-03-07
 * 
 * @copyright Copyright (c) 2026
 * @todo null pointer checks
 */
#include "dcc.h"
#include <stdlib.h>

/******************************************************************************
 * for python ctypes testing
 ******************************************************************************/

#ifdef PYTHON_TESTING

uint32_t
get_TR1_MIN() {
    return TR1_MIN;
}

uint32_t
get_TR1_MAX() {
    return TR1_MAX;
}

uint32_t
get_TR1D() {
    return TR1D;
}

uint32_t
get_TR0_MIN() {
    return TR0_MIN;
}

uint32_t
get_TR0_MAX() {
    return TR0_MAX;
}

uint32_t
get_DCC_BUF_LEN() {
    return DCC_BUF_LEN;
}

#endif                          /* #ifndef PYTHON_TESTING */

/******************************************************************************
 * private
 ******************************************************************************/

#ifndef PYTHON_TESTING
static inline int8_t
#else
int8_t                          /* to prevent indent from weirdly spacing explicit size types */
#endif                          /* #ifndef PYTHON_TESTING */
/**
 * @brief Identify a DCC bit given the index of a starting edge in a buffer.
 *
 * DCC is a bipolar asynchronous signal where bits are separated by rising or
 * falling edges. The bus has no unique notion of a rising edge, nor a
 * stateless way to determine which edge starts a bit. As such, this function
 * simply provides a means to determine the validity of a possible pair of
 * "halfbits" at a location in the buffer. Determining the correct location is
 * left to the state machine implementation.
 * 
 * @param device dcc_decoder_t * channel being evaluated
 * @param start_idx first index in the buffer of the three timestamps.
 * @return int8_t 1 if a 1 bit, 0 if a 0 bit, -2 if non-monotonic timestamps,
 *                -1 otherwise.
 * @note Must be fast for IRQs (time budget of ~10us). 
 */
parse_bit(dcc_decoder_t * device, uint8_t start_idx) {
    start_idx %= DCC_BUF_LEN;
    uint32_t t1 = device->buf[start_idx];
    uint32_t t2 = device->buf[(start_idx + 1) % DCC_BUF_LEN];
    uint32_t t3 = device->buf[(start_idx + 2) % DCC_BUF_LEN];

    if (t1 >= t2 || t2 >= t3) {
        return -2;
    }

    uint32_t dt1 = t2 - t1;
    uint32_t dt2 = t3 - t2;
    if (dt1 <= TR1_MAX && dt1 >= TR1_MIN && dt2 <= TR1_MAX
        && dt2 >= TR1_MIN && abs((int)(dt1 - dt2)) <= TR1D) {
        return 1;
    } else if (dt1 <= TR0_MAX
               && dt1 >= TR0_MIN && dt2 <= TR0_MAX && dt2 >= TR0_MIN) {
        return 0;
    }
    return -1;
}

static inline int8_t
parse_byte(dcc_decoder_t * device, uint8_t * dst, uint8_t base_idx) {
    uint8_t ctr = 7;
    int8_t bit = 0;
    for (uint8_t i = 0; i < 16; i += 2) {
        bit = parse_bit(device, (base_idx + i) % DCC_BUF_LEN);
        if (bit > -1) {
            *dst |= ((uint8_t) bit) << (ctr);
            ctr--;
        } else {
            device->state = ERROR;
            return -1;
        }
    }
    return 0;
}

static inline dcc_state_t
decode_packet(dcc_decoder_t * device) {
    uint8_t *dsts[3] = {
        &(device->packet.address),
        &(device->packet.instruction),
        &(device->packet.error_detection)
    };
    uint8_t base_idx = (device->r_idx + 2) % DCC_BUF_LEN;
    for (uint8_t i = 0; i < 3; i++) {
        if (parse_byte(device, dsts[i], base_idx) != 0) {
            return device->state;
        }
        base_idx += 16;
        base_idx %= DCC_BUF_LEN;
        if (i < 2) {
            if (parse_bit(device, base_idx) != 0) {
                return device->state;
            }
        } else if (parse_bit(device, base_idx) != 1) {
            return device->state;
        }
        base_idx += 2;
        base_idx %= DCC_BUF_LEN;
    }
    /* TODO: validate error detection */

    device->state = PACKET_RECEIVED;
    return device->state;
}

/******************************************************************************
 * public
 ******************************************************************************/

dcc_state_t
push_timestamp(dcc_decoder_t * device, uint32_t timestamp) {
    device->buf[device->w_idx++] = timestamp;
    device->w_idx %= DCC_BUF_LEN;

    /**
     * If awaiting packet start bit, check last 3 edge timestamps for a 0 bit.
     * The bit would end at w_idx-1, so start at w_idx-3, since w_idx was
     * incremented at the start of this function.
     *
     * Otherwise, increment the edge timestamp tracker
     */
    if (device->state == AWAITING_START_BIT) {
        uint8_t i = (device->w_idx + (-3 + DCC_BUF_LEN)) % DCC_BUF_LEN;
        if (parse_bit(device, i) == 0) {
            device->state = VALIDATING_PREAMBLE;
            device->r_idx = i;
        }
    } else {
        device->count++;
    }

    return device->state;
}

dcc_state_t
init_decoder(dcc_decoder_t * device) {
    reset_decoder(device);
    device->w_idx = 0;
    device->r_idx = 0;
    for (uint8_t i = 0; i < DCC_BUF_LEN; i++) {
        device->buf[i] = 0; /* TODO: does this even matter? */
    }
    return device->state;
}

dcc_state_t
reset_decoder(dcc_decoder_t * device) {
    device->state = AWAITING_START_BIT;
    device->count = 0;
    device->packet.address = 0;
    device->packet.instruction = 0;
    device->packet.error_detection = 0;
    return device->state;
}

dcc_state_t
step_decoder(dcc_decoder_t * device) {
    /* TODO: as a nice-to-have: check for next zero bytes in case of error */
    switch (device->state) {
        case AWAITING_START_BIT:
            break;              /* this state is handled in push_timestamp */
        case VALIDATING_PREAMBLE:
            device->state = AWAITING_DATA_BYTES;
            for (int8_t i = (DCC_BUF_LEN - 2); i > (DCC_BUF_LEN - 21); i -= 2) {
                if (1 != parse_bit(device, (device->r_idx + i) % DCC_BUF_LEN)) {
                    device->state = AWAITING_START_BIT; /* invalid preamble */
                    break;
                }
            }
            break;
        case AWAITING_DATA_BYTES:
            /* 3 sections, 9 bits per section, 2 edges per bit */
            if (device->count > 3 * 9 * 2 - 1) {
                /* TODO: find & validate end bit */
                /* TODO: if error, check for next preamble */
                device->state = DECODING_PACKET;
            }
            break;
        case DECODING_PACKET:
            decode_packet(device);
            break;
        case PACKET_RECEIVED:
        case ERROR:
            break;              /* TODO: what should these states do? */
    }
    return device->state;
}
