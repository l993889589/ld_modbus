/**
 * @file test_ld_modbus_rtu_framer.c
 * @brief Host regression tests for strict RTU T1.5/T3.5 receive framing.
 */

#include "ld_modbus_rtu_framer.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

/** @brief Initialize a small test framer with caller-owned storage. */
static void test_init(ld_modbus_rtu_framer_t *framer,
                      uint8_t *active,
                      uint8_t *ready,
                      uint16_t capacity,
                      uint32_t baud_rate,
                      uint8_t bits_per_char)
{
    assert(ld_modbus_rtu_framer_init(framer,
                                      active,
                                      ready,
                                      capacity,
                                      baud_rate,
                                      bits_per_char,
                                      1000000U));
}

/** @brief Verify calculated and fixed Modbus timing values. */
static void test_gap_calculation(void)
{
    uint32_t t15_us = 0U;
    uint32_t t35_us = 0U;

    assert(ld_modbus_rtu_char_time_us(9600U, 10U) == 1042U);
    ld_modbus_rtu_calculate_gaps(9600U, 10U, &t15_us, &t35_us);
    assert(t15_us == 1563U);
    assert(t35_us == 3646U);

    assert(ld_modbus_rtu_char_time_us(9600U, 11U) == 1146U);
    ld_modbus_rtu_calculate_gaps(9600U, 11U, &t15_us, &t35_us);
    assert(t15_us == 1719U);
    assert(t35_us == 4011U);

    ld_modbus_rtu_calculate_gaps(115200U, 10U, &t15_us, &t35_us);
    assert(t15_us == 750U);
    assert(t35_us == 1750U);

    ld_modbus_rtu_calculate_gaps(19200U, 10U, &t15_us, &t35_us);
    assert(t15_us == 782U);
    assert(t35_us == 1823U);
    ld_modbus_rtu_calculate_gaps(19201U, 10U, &t15_us, &t35_us);
    assert(t15_us == 750U);
    assert(t35_us == 1750U);
}

/** @brief Reject overlapping context and payload ownership regions. */
static void test_init_rejects_aliasing(void)
{
    ld_modbus_rtu_framer_t framer;
    uint8_t storage[32];

    assert(!ld_modbus_rtu_framer_init(&framer,
                                       storage,
                                       &storage[1],
                                       16U,
                                       9600U,
                                       10U,
                                       1000000U));
    assert(!ld_modbus_rtu_framer_init(&framer,
                                       (uint8_t *)&framer,
                                       storage,
                                       16U,
                                       9600U,
                                       10U,
                                       1000000U));
}

/** @brief Verify raw DWT-cycle timestamps and the exact T1.5 boundary. */
static void test_timing_engine_raw_ticks(void)
{
    ld_modbus_rtu_timing_t timing;
    uint32_t timestamp = 0xFFF00000UL;

    assert(ld_modbus_rtu_timing_init(&timing,
                                     9600U,
                                     10U,
                                     480000000U));
    assert(timing.char_ticks == 500000U);
    assert(timing.t15_ticks == 750000U);
    assert(timing.t35_ticks == 1750000U);

    assert(ld_modbus_rtu_timing_on_byte(&timing, timestamp) ==
           LD_MODBUS_RTU_BYTE_ACCEPT);
    assert(ld_modbus_rtu_timing_poll(&timing, timestamp - 1U) ==
           LD_MODBUS_RTU_POLL_NONE);
    assert(ld_modbus_rtu_timing_poll(
               &timing,
               timestamp + timing.t15_ticks + 1U) ==
           LD_MODBUS_RTU_POLL_NONE);

    timestamp += timing.char_ticks + timing.t15_ticks;
    assert(ld_modbus_rtu_timing_on_byte(&timing, timestamp) ==
           LD_MODBUS_RTU_BYTE_ACCEPT);
    timestamp += timing.char_ticks + timing.t15_ticks + 1U;
    assert(ld_modbus_rtu_timing_on_byte(&timing, timestamp) ==
           LD_MODBUS_RTU_BYTE_ABORT_AND_DISCARD);
    assert(timing.state == LD_MODBUS_RTU_TIMING_DISCARDING);
    assert(ld_modbus_rtu_timing_poll(&timing,
                                     timestamp + timing.t35_ticks) ==
           LD_MODBUS_RTU_POLL_NONE);
    assert(ld_modbus_rtu_timing_poll(
               &timing,
               timestamp + timing.char_ticks + timing.t35_ticks) ==
           LD_MODBUS_RTU_POLL_RECOVER);
    assert(timing.state == LD_MODBUS_RTU_TIMING_IDLE);

    /* A character that began just before T3.5 must reach on_byte before an
     * automatic commit can hide its illegal inter-character gap. */
    assert(ld_modbus_rtu_timing_init(&timing,
                                     9600U,
                                     10U,
                                     480000000U));
    timestamp = 1000U;
    assert(ld_modbus_rtu_timing_on_byte(&timing, timestamp) ==
           LD_MODBUS_RTU_BYTE_ACCEPT);
    timestamp += timing.char_ticks + timing.t35_ticks - 1U;
    assert(ld_modbus_rtu_timing_poll(&timing, timestamp) ==
           LD_MODBUS_RTU_POLL_NONE);
    assert(ld_modbus_rtu_timing_on_byte(&timing, timestamp) ==
           LD_MODBUS_RTU_BYTE_ABORT_AND_DISCARD);

    assert(ld_modbus_rtu_timing_init(&timing,
                                     115200U,
                                     10U,
                                     480000000U));
    assert(timing.char_ticks == 41667U);
    assert(timing.t15_ticks == 360000U);
    assert(timing.t35_ticks == 840000U);

    assert(ld_modbus_rtu_timing_init(&timing,
                                     9600U,
                                     10U,
                                     1000000U));
    assert(ld_modbus_rtu_timing_on_byte(&timing, 1000U) ==
           LD_MODBUS_RTU_BYTE_ACCEPT);
    assert(ld_modbus_rtu_timing_on_byte(&timing, 999U) ==
           LD_MODBUS_RTU_BYTE_ABORT_BAD_TIMESTAMP);
    assert(timing.state == LD_MODBUS_RTU_TIMING_DISCARDING);
}

/** @brief Verify a continuous stream is committed only after T3.5 silence. */
static void test_normal_frame(void)
{
    ld_modbus_rtu_framer_t framer;
    uint8_t active[16];
    uint8_t ready[16];
    uint8_t output[16];
    uint16_t output_length = 0U;
    uint32_t timestamp_us = 1000U;
    static const uint8_t expected[] = {1U, 3U, 0U, 0U};

    test_init(&framer, active, ready, sizeof(active), 9600U, 10U);
    for(uint32_t index = 0U; index < sizeof(expected); index++)
    {
        ld_modbus_rtu_framer_on_byte(&framer, expected[index], timestamp_us);
        timestamp_us += framer.timing.char_ticks;
    }

    ld_modbus_rtu_framer_poll(&framer,
                              framer.timing.last_byte_ticks +
                                  framer.timing.char_ticks +
                                  framer.timing.t35_ticks - 1U);
    assert(!ld_modbus_rtu_framer_take(&framer,
                                       output,
                                       sizeof(output),
                                       &output_length));
    ld_modbus_rtu_framer_poll(&framer,
                              framer.timing.last_byte_ticks +
                                  framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(ld_modbus_rtu_framer_take(&framer,
                                      output,
                                      sizeof(output),
                                      &output_length));
    assert(output_length == sizeof(expected));
    assert(memcmp(output, expected, sizeof(expected)) == 0);
    assert(framer.diag.frames_completed == 1U);
}

/** @brief Verify T1.5 violation discards all bytes until a later T3.5 gap. */
static void test_t15_discard_until_t35(void)
{
    ld_modbus_rtu_framer_t framer;
    uint8_t active[16];
    uint8_t ready[16];
    uint8_t output[16];
    uint16_t output_length = 0U;
    uint32_t timestamp_us = 1000U;

    test_init(&framer, active, ready, sizeof(active), 9600U, 10U);
    ld_modbus_rtu_framer_on_byte(&framer, 0x01U, timestamp_us);
    timestamp_us += framer.timing.char_ticks;
    ld_modbus_rtu_framer_on_byte(&framer, 0x03U, timestamp_us);

    timestamp_us += framer.timing.char_ticks +
                    framer.timing.t15_ticks + 1U;
    assert((timestamp_us - framer.timing.last_byte_ticks) <
           (framer.timing.char_ticks + framer.timing.t35_ticks));
    ld_modbus_rtu_framer_on_byte(&framer, 0xAAU, timestamp_us);
    assert(framer.diag.t15_violations == 1U);
    assert(framer.timing.state == LD_MODBUS_RTU_TIMING_DISCARDING);

    timestamp_us += framer.timing.char_ticks;
    ld_modbus_rtu_framer_on_byte(&framer, 0xBBU, timestamp_us);
    ld_modbus_rtu_framer_poll(&framer,
                              timestamp_us + framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(!ld_modbus_rtu_framer_take(&framer,
                                       output,
                                       sizeof(output),
                                       &output_length));
    assert(framer.timing.state == LD_MODBUS_RTU_TIMING_IDLE);

    timestamp_us += framer.timing.t35_ticks +
                    framer.timing.char_ticks;
    ld_modbus_rtu_framer_on_byte(&framer, 0x11U, timestamp_us);
    ld_modbus_rtu_framer_on_byte(&framer,
                                  0x22U,
                                  timestamp_us + framer.timing.char_ticks);
    ld_modbus_rtu_framer_poll(&framer,
                              timestamp_us + framer.timing.char_ticks +
                                  framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(ld_modbus_rtu_framer_take(&framer,
                                      output,
                                      sizeof(output),
                                      &output_length));
    assert(output_length == 2U && output[0] == 0x11U && output[1] == 0x22U);
}

/** @brief Verify an unpolled T3.5 boundary separates old and new frames. */
static void test_new_byte_after_t35(void)
{
    ld_modbus_rtu_framer_t framer;
    uint8_t active[8];
    uint8_t ready[8];
    uint8_t output[8];
    uint16_t output_length = 0U;
    uint32_t first_timestamp = 0xFFFFFF00UL;
    uint32_t next_timestamp;

    test_init(&framer, active, ready, sizeof(active), 115200U, 10U);
    ld_modbus_rtu_framer_on_byte(&framer, 0xA5U, first_timestamp);
    next_timestamp = first_timestamp + framer.timing.char_ticks +
                     framer.timing.t35_ticks;
    ld_modbus_rtu_framer_on_byte(&framer, 0x5AU, next_timestamp);

    assert(ld_modbus_rtu_framer_take(&framer,
                                      output,
                                      sizeof(output),
                                      &output_length));
    assert(output_length == 1U && output[0] == 0xA5U);
    ld_modbus_rtu_framer_poll(&framer,
                              next_timestamp + framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(ld_modbus_rtu_framer_take(&framer,
                                      output,
                                      sizeof(output),
                                      &output_length));
    assert(output_length == 1U && output[0] == 0x5AU);
}

/** @brief Verify overflow discards the invalid stream until T3.5 silence. */
static void test_overflow_recovery(void)
{
    ld_modbus_rtu_framer_t framer;
    uint8_t active[2];
    uint8_t ready[2];
    uint8_t output[2];
    uint16_t output_length = 0U;

    test_init(&framer, active, ready, sizeof(active), 115200U, 10U);
    ld_modbus_rtu_framer_on_byte(&framer, 1U, 100U);
    ld_modbus_rtu_framer_on_byte(&framer, 2U, 200U);
    ld_modbus_rtu_framer_on_byte(&framer, 3U, 300U);
    assert(framer.diag.overflow == 1U);
    assert(framer.timing.state == LD_MODBUS_RTU_TIMING_DISCARDING);
    ld_modbus_rtu_framer_poll(&framer,
                              300U + framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(!ld_modbus_rtu_framer_take(&framer,
                                       output,
                                       sizeof(output),
                                       &output_length));
}

/** @brief Verify the legal 256-byte RTU ADU and reject byte 257. */
static void test_protocol_maximum_capacity(void)
{
    ld_modbus_rtu_framer_t framer;
    ld_modbus_rtu_frame_view_t view;
    uint8_t active[LD_MODBUS_RTU_MAX_ADU_LENGTH];
    uint8_t ready[LD_MODBUS_RTU_MAX_ADU_LENGTH];
    uint32_t timestamp = 1000U;
    uint32_t index;

    test_init(&framer,
              active,
              ready,
              sizeof(active),
              115200U,
              10U);
    for(index = 0U; index < LD_MODBUS_RTU_MAX_ADU_LENGTH; index++)
    {
        ld_modbus_rtu_framer_on_byte(&framer, (uint8_t)index, timestamp);
        timestamp += framer.timing.char_ticks;
    }
    ld_modbus_rtu_framer_poll(&framer,
                              framer.timing.last_byte_ticks +
                                  framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(ld_modbus_rtu_framer_claim(&framer, &view));
    assert(view.length == LD_MODBUS_RTU_MAX_ADU_LENGTH);
    assert(ld_modbus_rtu_framer_release(&framer, &view));

    timestamp += framer.timing.t35_ticks;
    for(index = 0U; index <= LD_MODBUS_RTU_MAX_ADU_LENGTH; index++)
    {
        ld_modbus_rtu_framer_on_byte(&framer, (uint8_t)index, timestamp);
        timestamp += framer.timing.char_ticks;
    }
    assert(framer.diag.overflow == 1U);
    assert(framer.timing.state == LD_MODBUS_RTU_TIMING_DISCARDING);
    ld_modbus_rtu_framer_poll(&framer,
                              framer.timing.last_byte_ticks +
                                  framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(!ld_modbus_rtu_framer_claim(&framer, &view));
}

/** @brief UART receive errors invalidate only the current partial stream. */
static void test_receive_error_recovery(void)
{
    ld_modbus_rtu_framer_t framer;
    uint8_t active[16];
    uint8_t ready[16];
    uint8_t output[16];
    uint16_t output_length = 0U;
    uint32_t timestamp = 1000U;

    test_init(&framer, active, ready, sizeof(active), 9600U, 10U);
    ld_modbus_rtu_framer_on_byte(&framer, 0x01U, timestamp);
    timestamp += framer.timing.char_ticks;
    ld_modbus_rtu_framer_on_byte(&framer, 0x03U, timestamp);
    timestamp += framer.timing.char_ticks;
    ld_modbus_rtu_framer_on_error(&framer, timestamp);
    assert(framer.diag.rx_errors == 1U);
    assert(framer.diag.discarded_bytes == 2U);
    assert(framer.timing.state == LD_MODBUS_RTU_TIMING_DISCARDING);

    ld_modbus_rtu_framer_poll(&framer,
                              timestamp + framer.timing.char_ticks +
                                  framer.timing.t35_ticks - 1U);
    assert(framer.timing.state == LD_MODBUS_RTU_TIMING_DISCARDING);
    ld_modbus_rtu_framer_poll(&framer,
                              timestamp + framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(framer.timing.state == LD_MODBUS_RTU_TIMING_IDLE);
    assert(!ld_modbus_rtu_framer_take(&framer,
                                       output,
                                       sizeof(output),
                                       &output_length));

    timestamp += framer.timing.char_ticks + framer.timing.t35_ticks;
    ld_modbus_rtu_framer_on_byte(&framer, 0x11U, timestamp);
    ld_modbus_rtu_framer_poll(&framer,
                              timestamp + framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(ld_modbus_rtu_framer_take(&framer,
                                      output,
                                      sizeof(output),
                                      &output_length));
    assert(output_length == 1U && output[0] == 0x11U);
}

/** @brief Verify a claimed frame is stable and backpressure drops the newer one. */
static void test_zero_copy_claim_is_stable(void)
{
    ld_modbus_rtu_framer_t framer;
    ld_modbus_rtu_frame_view_t view;
    uint8_t active[8];
    uint8_t ready[8];
    uint32_t timestamp = 1000U;

    test_init(&framer, active, ready, sizeof(active), 115200U, 10U);
    ld_modbus_rtu_framer_on_byte(&framer, 0x11U, timestamp);
    ld_modbus_rtu_framer_poll(&framer,
                              timestamp + framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(ld_modbus_rtu_framer_claim(&framer, &view));
    assert(view.length == 1U && view.data[0] == 0x11U);

    timestamp += framer.timing.t35_ticks + framer.timing.char_ticks;
    ld_modbus_rtu_framer_on_byte(&framer, 0x22U, timestamp);
    ld_modbus_rtu_framer_poll(&framer,
                              timestamp + framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(view.length == 1U && view.data[0] == 0x11U);
    assert(framer.diag.dropped_while_ready == 1U);
    assert(!ld_modbus_rtu_framer_reset(&framer));
    assert(ld_modbus_rtu_framer_release(&framer, &view));

    timestamp += framer.timing.t35_ticks + framer.timing.char_ticks;
    ld_modbus_rtu_framer_on_byte(&framer, 0x33U, timestamp);
    ld_modbus_rtu_framer_poll(&framer,
                              timestamp + framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(ld_modbus_rtu_framer_claim(&framer, &view));
    assert(view.length == 1U && view.data[0] == 0x33U);
    assert(ld_modbus_rtu_framer_release(&framer, &view));
}

/** @brief Reject view/output aliases without damaging receiver state. */
static void test_consumer_aliases_are_rejected(void)
{
    ld_modbus_rtu_framer_t framer;
    ld_modbus_rtu_frame_view_t view;
    uint8_t active[16];
    uint8_t ready[16];
    uint8_t output[16];
    uint16_t output_length = 0U;

    test_init(&framer, active, ready, sizeof(active), 115200U, 10U);
    ld_modbus_rtu_framer_on_byte(&framer, 0x5AU, 1000U);
    ld_modbus_rtu_framer_poll(&framer,
                              1000U + framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(!ld_modbus_rtu_framer_claim(
        &framer,
        (ld_modbus_rtu_frame_view_t *)&framer));
    assert(!ld_modbus_rtu_framer_claim(
        &framer,
        (ld_modbus_rtu_frame_view_t *)framer.ready_buffer));
    assert(!ld_modbus_rtu_framer_take(&framer,
                                       framer.ready_buffer,
                                       sizeof(ready),
                                       &output_length));
    assert(!ld_modbus_rtu_framer_take(
        &framer,
        output,
        sizeof(output),
        (uint16_t *)&framer.ready_length));
    assert(ld_modbus_rtu_framer_claim(&framer, &view));
    assert(view.length == 1U && view.data[0] == 0x5AU);
    assert(ld_modbus_rtu_framer_release(&framer, &view));
}

/** @brief Backpressure and a small output never consume the older frame. */
static void test_ready_backpressure_and_small_take(void)
{
    ld_modbus_rtu_framer_t framer;
    ld_modbus_rtu_frame_view_t view;
    uint8_t active[8];
    uint8_t ready[8];
    uint8_t output[1];
    uint16_t output_length = 99U;
    uint32_t timestamp = 1000U;

    test_init(&framer, active, ready, sizeof(active), 115200U, 10U);
    ld_modbus_rtu_framer_on_byte(&framer, 0x11U, timestamp);
    ld_modbus_rtu_framer_on_byte(&framer,
                                  0x22U,
                                  timestamp + framer.timing.char_ticks);
    ld_modbus_rtu_framer_poll(&framer,
                              timestamp + (2U * framer.timing.char_ticks) +
                                  framer.timing.t35_ticks);

    timestamp += (3U * framer.timing.char_ticks) +
                 framer.timing.t35_ticks;
    ld_modbus_rtu_framer_on_byte(&framer, 0x33U, timestamp);
    ld_modbus_rtu_framer_poll(&framer,
                              timestamp + framer.timing.char_ticks +
                                  framer.timing.t35_ticks);
    assert(framer.diag.dropped_while_ready == 1U);

    assert(!ld_modbus_rtu_framer_take(&framer,
                                       output,
                                       sizeof(output),
                                       &output_length));
    assert(output_length == 0U);
    assert(ld_modbus_rtu_framer_claim(&framer, &view));
    assert(view.length == 2U && view.data[0] == 0x11U &&
           view.data[1] == 0x22U);
    assert(ld_modbus_rtu_framer_release(&framer, &view));
}

/** @brief Run all host-side RTU framer regression cases. */
int main(void)
{
    test_gap_calculation();
    test_init_rejects_aliasing();
    test_timing_engine_raw_ticks();
    test_normal_frame();
    test_t15_discard_until_t35();
    test_new_byte_after_t35();
    test_overflow_recovery();
    test_protocol_maximum_capacity();
    test_receive_error_recovery();
    test_zero_copy_claim_is_stable();
    test_consumer_aliases_are_rejected();
    test_ready_backpressure_and_small_take();
    puts("ld_modbus RTU framer tests passed");
    return 0;
}
