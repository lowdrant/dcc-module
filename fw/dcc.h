/**
 * @file dcc.h
 * @author lowdrant
 * @brief C library for decoding Digital Command Control (DCC) signals.
 *
 * DCC is a niche asynchronous serial communication protocol where each bit is
 * encoded as two pulse widths. Since there are no standard peripherals to
 * decode DCC, this library implements both pulse width buffer handling and
 * packet decoding/validation.
 *
 * @note Electrical standard: https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.1_electrical_standards_for_digital_command_control.pdf
 * @note Packet standard: https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-92-2004-07.pdf
 *
 * @version 0.1
 * @date 2026-03-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _DCC_H
#define _DCC_H
#include <stdint.h>

#define TR1_MIN ((uint32_t) 52) /*!< Minimum '1' halfbit width in us. */
#define TR1_MAX ((uint32_t) 64) /*!< Maximum '1' halfbit width in us. */
#define TR1D ((uint32_t) 6)     /*!< Maximum difference in '1' halfbit widths within a single bit in us */
#define TR0_MIN ((uint32_t) 90) /*!< Minimum '0' halfbit width in us. */
#define TR0_MAX ((uint32_t) 10000)      /*!< Maximum '0' halfbit width in us. */
#define DCC_BUF_LEN ((uint8_t) 64)      /*!< Smallest power of two that holds a packet */

/**
 * @brief Valid DCC decoder states. 
 */
typedef enum {
    AWAITING_START_BIT,         /*!<  The decoder waits for a start bit before attempting parsing. */
    VALIDATING_PREAMBLE,        /*!< A valid start bit has been received. The program should validate the preamble at its earliest convenience */
    AWAITING_DATA_BYTES,        /*!< @todo */
    DECODING_PACKET,            /*!< Enough data received, decode soon. */
    PACKET_RECEIVED,            /*!< A valid packet was received and is ready for action. */
    ERROR,                      /*!< The received packet was invalid. TODO: ignore this state entirely? */
} dcc_state_t;

/**
 * @brief Stores decoded DCC packet data.
 */
typedef struct {
    uint8_t address;            /*!< Address of the decoder being commanded by the bus. */
    uint8_t instruction;        /*!< The transmitted command. */
    uint8_t error_detection;    /*!< Error detection byte. Should be address ^ instruction */
} dcc_packet_t;

/**
 * @brief DCC decoder data type.
 *
 * @see dcc_state_t
 * @see dcc_packet_t
 * @see init_decoder
 * @see push_timestamp
 * @see validate_preamble
 */
typedef struct {
    dcc_packet_t packet;        /*!< Decoded packet if state == PACKET_RECEIVED. */
    dcc_state_t state;          /*!< Current decoder state. */
    uint32_t buf[DCC_BUF_LEN];  /*!< Circular buffer storing signal edge crossing times. */
    uint8_t w_idx;              /*!< Next index to be written in buffer. */
    uint8_t r_idx;              /*!< The ending (third) edge of the packet start bit. */
    uint8_t count;              /*!< Number of packet timestamps in buffer. */
} dcc_decoder_t;

/**
 * @brief Push edge crossing time onto decoder buffer and handle any tasks that
 *        should be performed with each edge crossing. Intended to be called
 *        inside an IRQ.
 *
 * This function pus the circular buffer, checks for the packet start
 * bit, and counts the number of post-start-bit edge crossings. The packet start
 * bit serves to time-align the packet decoding process. Since we may not be
 * time-aligned, checking for the start bit every edge reduces processing
 * time spikes. Further, since the buffer is circular, the number of edges
 * must be explicitly counted to know the number of bits that have arrived
 * post-start-bit.
 * 
 * @param device dcc_decoder_t * on the RX line.
 * @param timestamp timestamp of edge crossing.
 * @return dcc_state_t 
 */
dcc_state_t push_timestamp(dcc_decoder_t * device, uint32_t timestamp);

/**
 * @brief Initialize decoder. Sets all members to 0.
 * 
 * @param device dcc_decoder_t * device to initialize.
 * @return dcc_state_t should be `AWAITING_START_BIT`.
 */
dcc_state_t init_decoder(dcc_decoder_t * device);

/* TODO */
dcc_state_t reset_decoder(dcc_decoder_t * device);

/**
 * @brief Advance decoder state machine if allowed and perform relevant
 *        processing. This function should be called several times a
 *        millisecond.
 * 
 * This function implements the decoder state machine once a packet start bit
 * has been detected -- start bit detection occurs in push_timestamp, which is
 * intended to be put inside an interrupt. This function is intended to run in
 * the mainloop. It handles all processing to advance the decoding state
 * machine and decode the current packet.
 *
 * @param device dcc_decoder_t * device to 
 * @return dcc_state_t device 
 */
dcc_state_t step_decoder(dcc_decoder_t * device);

#endif                          /* #ifndef _DCC_H */
