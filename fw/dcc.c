/**
 * @file dcc.c
 * @author lowdrant
 * @brief Implementations for interface described in dcc.h.
 *
 * Supports `-DPYTHON_TESTING` define flag for unit testing against Python.
 *
 * @version 0.1
 * @date 2026-03-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "dcc.h"
#include <stdlib.h>
#include <stdint.h>

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
static inline uint8_t
#else
uint8_t
#endif                          /* #ifndef PYTHON_TESTING */
/**
 * @brief Increment an index forwards or backwards along a DCC buffer.
 * 
 * @param i uint8_t the buffer index to be incremented.
 * @param d int8_t the amount to increment i.
 * @return uint8_t i incremented by d.
 */
increment_index(uint8_t i, int8_t d) {
    d %= DCC_BUF_LEN;           /* put d within 1 wrap of buf */
    return (i + (d + DCC_BUF_LEN)) % DCC_BUF_LEN;
}

#ifndef PYTHON_TESTING
static inline int8_t
#else
int8_t                          /* to prevent indent from weirdly spaceing explicit size types */
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
    uint32_t t2 = device->buf[increment_index(start_idx, 1)];
    uint32_t t3 = device->buf[increment_index(start_idx, 2)];

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

/******************************************************************************
 * public
 ******************************************************************************/

dcc_state_t
push_timestamp(dcc_decoder_t * device, uint32_t timestamp) {
    device->buf[device->w_idx++] = timestamp;
    device->w_idx %= DCC_BUF_LEN;

    /* check for packet start bit, if applicable */
    if (device->state == AWAITING_START_BIT) {
        uint8_t i = increment_index(device->w_idx, -3);
        if (parse_bit(device, i) == 0) {
            device->state = VALIDATING_PREAMBLE;
            device->r_idx = i;
        }
    }

    return device->state;
}

dcc_state_t
init_decoder(dcc_decoder_t * device) {
    device->state = AWAITING_START_BIT;
    device->w_idx = 0;
    device->r_idx = 0;
    device->packet.address = 0;
    device->packet.instruction = 0;
    device->packet.error_detection = 0;
    for (uint8_t i = 0; i < DCC_BUF_LEN; i++) {
        device->buf[i] = 0;
    }
    return device->state;
}

dcc_state_t
validate_preamble(dcc_decoder_t * device) {
    if (device->state != VALIDATING_PREAMBLE) {
        device->state = ERROR;
        return device->state;
    }
    device->state = AWAITING_DATA_BYTES;        /* preemptive assignment */
    for (int8_t i = -2; i > -21; i -= 2) {
        if (1 != parse_bit(device, increment_index(device->r_idx, i))) {
            device->state = AWAITING_START_BIT; /* invalid preamble */
            break;
        }
    }
    return device->state;
}
