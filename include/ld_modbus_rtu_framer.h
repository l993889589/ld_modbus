/**
 * @file ld_modbus_rtu_framer.h
 * @brief Platform-independent Modbus RTU T1.5/T3.5 receive framer.
 *
 * This optional module belongs to the ld_modbus distribution but is not a
 * dependency of the RTU/TCP codec or client/server core. The platform port
 * supplies baud information, byte-completion timestamps, and current time.
 */

#ifndef LD_MODBUS_RTU_FRAMER_H
#define LD_MODBUS_RTU_FRAMER_H

#include "ld_modbus.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief RTU timing state. Treat values as read-only after initialization. */
typedef enum
{
    LD_MODBUS_RTU_TIMING_IDLE = 0,
    LD_MODBUS_RTU_TIMING_RECEIVING,
    LD_MODBUS_RTU_TIMING_DISCARDING
} ld_modbus_rtu_timing_state_t;

/** @brief Action selected before storing one newly completed character. */
typedef enum
{
    LD_MODBUS_RTU_BYTE_ACCEPT = 0,
    LD_MODBUS_RTU_BYTE_COMMIT_AND_ACCEPT,
    LD_MODBUS_RTU_BYTE_ABORT_AND_DISCARD,
    LD_MODBUS_RTU_BYTE_ABORT_BAD_TIMESTAMP,
    LD_MODBUS_RTU_BYTE_DISCARD
} ld_modbus_rtu_byte_action_t;

/** @brief Action selected by a silence poll. */
typedef enum
{
    LD_MODBUS_RTU_POLL_NONE = 0,
    LD_MODBUS_RTU_POLL_COMMIT,
    LD_MODBUS_RTU_POLL_RECOVER
} ld_modbus_rtu_poll_action_t;

/**
 * @brief Buffer-independent T1.5/T3.5 timing engine.
 *
 * Timestamps are wrapping 32-bit ticks from one caller-owned clock. Raw DWT
 * cycles, a 1 MHz hardware timer, or another free-running source are valid.
 * All calls that touch one object must be serialized by the caller.
 */
typedef struct
{
    uint32_t timestamp_hz;
    uint32_t char_ticks;
    uint32_t t15_ticks;
    uint32_t t35_ticks;
    uint32_t last_byte_ticks;
    ld_modbus_rtu_timing_state_t state;
} ld_modbus_rtu_timing_t;

/** @brief Receive-framing diagnostics retained across resets. */
typedef struct
{
    uint32_t frames_completed;
    uint32_t t15_violations;
    uint32_t overflow;
    uint32_t dropped_while_ready;
    uint32_t discarded_bytes;
    uint32_t timestamp_errors;
    uint32_t rx_errors;
} ld_modbus_rtu_framer_diag_t;

/** @brief Immutable zero-copy view of one completed RTU ADU. */
typedef struct
{
    const uint8_t *data;
    uint16_t length;
    uint32_t generation;
} ld_modbus_rtu_frame_view_t;

/** @brief Caller-owned RTU receive state and double-buffer bindings. */
typedef struct
{
    uint8_t *active_buffer;
    uint8_t *ready_buffer;
    uint16_t capacity;
    uint16_t active_length;
    uint16_t ready_length;
    uint32_t ready_generation;
    ld_modbus_rtu_timing_t timing;
    bool active_open;
    bool ready_available;
    bool ready_claimed;
    ld_modbus_rtu_framer_diag_t diag;
} ld_modbus_rtu_framer_t;

/**
 * @brief Calculate one complete serial-character duration, rounded up.
 * @param baud_rate Current UART baud rate in bits per second.
 * @param bits_per_char Total start, data, parity, and stop bits per character.
 * @return Character duration in microseconds, or zero for invalid arguments.
 */
uint32_t ld_modbus_rtu_char_time_us(uint32_t baud_rate,
                                    uint8_t bits_per_char);

/**
 * @brief Calculate the Modbus RTU T1.5 and T3.5 silent intervals.
 * @param baud_rate Current UART baud rate in bits per second.
 * @param bits_per_char Total serial bits per character.
 * @param t15_us Receives T1.5 in microseconds.
 * @param t35_us Receives T3.5 in microseconds.
 * @note Baud rates above 19200 use the recommended fixed 750/1750 us values.
 */
void ld_modbus_rtu_calculate_gaps(uint32_t baud_rate,
                                  uint8_t bits_per_char,
                                  uint32_t *t15_us,
                                  uint32_t *t35_us);

/**
 * @brief Initialize a buffer-independent RTU timing engine.
 * @param timestamp_hz Frequency of the wrapping timestamp clock in hertz.
 * @return True when all arguments and derived thresholds are valid.
 * @note All relevant elapsed intervals must be less than 2^31 clock ticks.
 */
bool ld_modbus_rtu_timing_init(ld_modbus_rtu_timing_t *timing,
                               uint32_t baud_rate,
                               uint8_t bits_per_char,
                               uint32_t timestamp_hz);

/**
 * @brief Classify one byte using its end-of-character timestamp.
 * @note T1.5 is checked only when the next byte arrives. Silence crossing
 * T1.5 alone is not an error because it may continue to a legal T3.5 end.
 */
ld_modbus_rtu_byte_action_t ld_modbus_rtu_timing_on_byte(
    ld_modbus_rtu_timing_t *timing,
    uint32_t timestamp_ticks);

/**
 * @brief Invalidate the current stream after a UART/storage receive error.
 * @param timestamp_ticks Timestamp of the error observation on the same clock.
 * @note Recovery requires a later T3.5 silent interval, just like a malformed
 * RTU stream. A completed frame already waiting for the consumer is retained.
 */
void ld_modbus_rtu_timing_on_error(ld_modbus_rtu_timing_t *timing,
                                   uint32_t timestamp_ticks);

/**
 * @brief Classify current silence using the same wrapping timestamp clock.
 * @note Because the input timestamps mark character completion, automatic
 * commit waits T3.5 plus one character time. This prevents a character that
 * began before T3.5 but has not completed yet from being split into a new
 * frame. The wire-level frame boundary remains T3.5.
 * A delta greater than 2^31-1 ticks is treated as an older/ambiguous sample,
 * not as a valid long silence.
 */
ld_modbus_rtu_poll_action_t ld_modbus_rtu_timing_poll(
    ld_modbus_rtu_timing_t *timing,
    uint32_t now_ticks);

/** @brief Enter discard-until-T3.5 after an external storage failure. */
void ld_modbus_rtu_timing_force_discard(ld_modbus_rtu_timing_t *timing);

/** @brief Return the timing engine to idle without changing its thresholds. */
void ld_modbus_rtu_timing_reset(ld_modbus_rtu_timing_t *timing);

/**
 * @brief Initialize a framer with two caller-owned buffers of equal capacity.
 * @param timestamp_hz Frequency of the wrapping end-of-character clock.
 * @return True when arguments and calculated timing values are valid.
 * @note capacity may be smaller than the Modbus RTU maximum when the product
 * intentionally supports only smaller ADUs, but it may never exceed it.
 */
bool ld_modbus_rtu_framer_init(ld_modbus_rtu_framer_t *ctx,
                               uint8_t *active_buffer,
                               uint8_t *ready_buffer,
                               uint16_t capacity,
                               uint32_t baud_rate,
                               uint8_t bits_per_char,
                               uint32_t timestamp_hz);

/**
 * @brief Abort a partial stream and replace baud/format timing.
 * @note A previously completed frame remains available to the consumer.
 */
bool ld_modbus_rtu_framer_reconfigure(ld_modbus_rtu_framer_t *ctx,
                                      uint32_t baud_rate,
                                      uint8_t bits_per_char,
                                      uint32_t timestamp_hz);

/**
 * @brief Feed one byte and its end-of-character timestamp.
 * @param timestamp_ticks Wrapping timestamp captured when this character
 * finished reception. Unsigned subtraction makes 32-bit timer wrap safe.
 * @note A silence greater than T1.5 but less than T3.5 discards the complete
 * invalid stream until a new T3.5 silent interval is observed.
 */
void ld_modbus_rtu_framer_on_byte(ld_modbus_rtu_framer_t *ctx,
                                  uint8_t byte,
                                  uint32_t timestamp_ticks);

/**
 * @brief Abort the partial frame after parity/framing/noise/overrun loss.
 * @note Call once per receive-error event. The exact hardware error flags stay
 * in the platform diagnostics; this portable layer counts invalidated events.
 */
void ld_modbus_rtu_framer_on_error(ld_modbus_rtu_framer_t *ctx,
                                   uint32_t timestamp_ticks);

/**
 * @brief Commit an open frame after T3.5 silence from the last completed byte.
 * @param now_ticks Current wrapping time from the same clock source.
 */
void ld_modbus_rtu_framer_poll(ld_modbus_rtu_framer_t *ctx,
                               uint32_t now_ticks);

/**
 * @brief Claim the completed RTU ADU without copying it.
 * @return True when a frame was claimed.
 * @note The producer will never overwrite a claimed view. A second complete
 * frame is dropped while the ready slot is occupied. Serialize the short
 * claim/release calls with producer boundary transitions; payload processing
 * between them does not need to hold the UART critical section.
 */
bool ld_modbus_rtu_framer_claim(ld_modbus_rtu_framer_t *ctx,
                                ld_modbus_rtu_frame_view_t *view);

/** @brief Consume and release a view returned by framer_claim(). */
bool ld_modbus_rtu_framer_release(ld_modbus_rtu_framer_t *ctx,
                                  ld_modbus_rtu_frame_view_t *view);

/**
 * @brief Copy and consume one completed frame.
 * @return True when a frame was copied; false when none is ready or output is
 * too small. A too-small output does not consume the queued frame.
 */
bool ld_modbus_rtu_framer_take(ld_modbus_rtu_framer_t *ctx,
                               uint8_t *out,
                               uint16_t out_capacity,
                               uint16_t *out_length);

/**
 * @brief Drop open and completed data while retaining diagnostics and timing.
 * @return False when a consumer currently holds a claimed frame.
 */
bool ld_modbus_rtu_framer_reset(ld_modbus_rtu_framer_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* LD_MODBUS_RTU_FRAMER_H */
