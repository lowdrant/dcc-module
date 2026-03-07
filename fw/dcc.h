/** dcc.h
  *
  * Stores configuration for recieving Digital Command Control (DCC) signals
  *
  * https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-9.1_electrical_standards_for_digital_command_control.pdf
  *
  * processing happens in 2 steps: validating preamble, then decoding packet.
  * this should, on average, reduce the number of instructions executed when
  * decoding a packet. this is captured in a state machine. the process should go
  * - wait for startbit
  * - once start bit, validate preamble
  * - if preamble, wait for data
  * - if data valid, ready, else error
  * invalid preambles just reset preamble detection
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
 * @todo describe state machine progression
 *
 * @see dcc_state_t
 * @see dcc_packet_t
 * @see init_decoder
 * @see push_timestamp
 * @see validate_preamble
 */
typedef struct {
    dcc_packet_t packet;        /*!< Decoded packet if state == PACKET_RECEIVED. */
    dcc_state_t state;          /*!< Decoder current state. */
    uint32_t buf[DCC_BUF_LEN];  /*!< Circular buffer storing signal edge crossing times. */
    uint8_t w_idx;              /*!< Next index to be written in buffer. */
    uint8_t r_idx;              /*!< Index to read from buffer. Only valid when TODO */
    uint8_t count;              /*!< Number of packet timestamps in buffer */
} dcc_decoder_t;

/**
 * @brief Push edge crossing time onto decoder buffer. Written to be called
 *        inside an IRQ.
 *
 * This is the hardware -> state machine interface for the decoder. Its core
 * job is to push timestamps onto the circular buffer. Data will be overwritten
 * in the event of an "overflow." If the decoder state is `AWAITING_START_BIT`,
 * and a start bit is detected, this function will update the state to
 * `VALIDATING_PREAMBLE` and synchronize `device->r_idx` to the starting edge
 * of the start bit.
 * 
 * @param device dcc_decoder_t * on the RX line.
 * @param timestamp timestamp of edge crossing.
 * @return dcc_state_t 
 */
dcc_state_t new_timestamp(dcc_decoder_t * device, uint32_t timestamp);

/**
 * @brief Initialize decoder. Sets all members to 0.
 * 
 * @param device dcc_decoder_t * device to initialize.
 * @return dcc_state_t should be `AWAITING_START_BIT`.
 */
dcc_state_t init_decoder(dcc_decoder_t * device);

/**
 * @brief Validate recieved preamble. Updates device->state based on validation
 *        result.
 *
 * This function should be called shortly after the device state becomes
 * `VALIDATING_PREAMBLE`. If the preamble is valid, the state becomes
 * `AWAITING_DATA_BYTES` else `AWAITING_START_BIT`. If device state is anything
 * other than `VALIDATING_PREAMBLE` at the start of this function, the device
 * state is set to `ERROR`.
 * 
 * @param device dcc_decoder_t * device that has received start bit.
 * @return dcc_state_t device state after validation.
 */
dcc_state_t validate_preamble(dcc_decoder_t * device);

#endif                          /* #ifndef _DCC_H */
