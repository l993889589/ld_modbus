/**
 * @file ld_modbus_rtu_framer.c
 * @brief Platform-independent Modbus RTU T1.5/T3.5 framing implementation.
 */

#include "ld_modbus_rtu_framer.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define LD_MODBUS_RTU_FIXED_GAP_BAUDRATE (19200U)
#define LD_MODBUS_RTU_FIXED_T15_US       (750U)
#define LD_MODBUS_RTU_FIXED_T35_US       (1750U)
#define LD_MODBUS_US_PER_SECOND          (1000000ULL)
#define LD_MODBUS_WRAP_SAFE_MAX_TICKS    (0x7FFFFFFFUL)

/** @brief Divide a 64-bit numerator by a nonzero denominator, rounding up. */
static uint32_t ld_modbus_div_round_up_u64(uint64_t numerator,
                                           uint64_t denominator)
{
    return (uint32_t)((numerator + denominator - 1ULL) / denominator);
}

/** @brief Calculate ceil(value * multiplier / divisor) without 64-bit math. */
static uint32_t ld_modbus_mul_div_round_up_u32(uint32_t value,
                                              uint32_t multiplier,
                                              uint32_t divisor)
{
    uint32_t quotient;
    uint32_t remainder;
    uint32_t whole;
    uint32_t fraction;
    uint32_t rounded_fraction;

    if((value == 0U) || (multiplier == 0U) || (divisor == 0U))
    {
        return 0U;
    }
    quotient = value / divisor;
    remainder = value % divisor;
    if((quotient > (UINT32_MAX / multiplier)) ||
       (remainder > (UINT32_MAX / multiplier)))
    {
        return 0U;
    }
    whole = quotient * multiplier;
    fraction = remainder * multiplier;
    rounded_fraction = fraction / divisor;
    if((fraction % divisor) != 0U)
    {
        rounded_fraction++;
    }
    if(whole > (UINT32_MAX - rounded_fraction))
    {
        return 0U;
    }
    return whole + rounded_fraction;
}

/** @brief Test that two non-empty caller-owned memory regions are disjoint. */
static bool ld_modbus_rtu_regions_are_disjoint(const void *first,
                                               size_t first_size,
                                               const void *second,
                                               size_t second_size)
{
    uintptr_t first_begin = (uintptr_t)first;
    uintptr_t second_begin = (uintptr_t)second;
    uintptr_t first_end;
    uintptr_t second_end;

    if((first == NULL) || (second == NULL) ||
       (first_size == 0U) || (second_size == 0U) ||
       (first_size > (size_t)(UINTPTR_MAX - first_begin)) ||
       (second_size > (size_t)(UINTPTR_MAX - second_begin)))
    {
        return false;
    }
    first_end = first_begin + (uintptr_t)first_size;
    second_end = second_begin + (uintptr_t)second_size;
    return (first_end <= second_begin) || (second_end <= first_begin);
}

/** @brief Publish by swapping buffers, or drop when ready remains occupied. */
static void ld_modbus_rtu_framer_commit_active(ld_modbus_rtu_framer_t *ctx)
{
    uint8_t *standby_buffer;

    if((ctx == NULL) || !ctx->active_open || (ctx->active_length == 0U))
    {
        return;
    }

    if(ctx->ready_available)
    {
        ctx->diag.dropped_while_ready++;
        ctx->diag.discarded_bytes += ctx->active_length;
    }
    else
    {
        standby_buffer = ctx->ready_buffer;
        ctx->ready_buffer = ctx->active_buffer;
        ctx->active_buffer = standby_buffer;
        ctx->ready_length = ctx->active_length;
        ctx->ready_generation++;
        if(ctx->ready_generation == 0U)
        {
            ctx->ready_generation++;
        }
        ctx->ready_available = true;
        ctx->ready_claimed = false;
        ctx->diag.frames_completed++;
    }

    ctx->active_length = 0U;
    ctx->active_open = false;
}

/** @brief Calculate one complete serial-character duration, rounded up. */
uint32_t ld_modbus_rtu_char_time_us(uint32_t baud_rate,
                                    uint8_t bits_per_char)
{
    if((baud_rate == 0U) || (bits_per_char == 0U))
    {
        return 0U;
    }

    return ld_modbus_div_round_up_u64((uint64_t)bits_per_char *
                                          LD_MODBUS_US_PER_SECOND,
                                      baud_rate);
}

/** @brief Calculate the Modbus RTU T1.5 and T3.5 silent intervals. */
void ld_modbus_rtu_calculate_gaps(uint32_t baud_rate,
                                  uint8_t bits_per_char,
                                  uint32_t *t15_us,
                                  uint32_t *t35_us)
{
    uint64_t one_char_numerator;

    if((t15_us == NULL) || (t35_us == NULL))
    {
        return;
    }

    *t15_us = 0U;
    *t35_us = 0U;
    if((baud_rate == 0U) || (bits_per_char == 0U))
    {
        return;
    }

    if(baud_rate > LD_MODBUS_RTU_FIXED_GAP_BAUDRATE)
    {
        *t15_us = LD_MODBUS_RTU_FIXED_T15_US;
        *t35_us = LD_MODBUS_RTU_FIXED_T35_US;
        return;
    }

    one_char_numerator =
        (uint64_t)bits_per_char * LD_MODBUS_US_PER_SECOND;
    *t15_us = ld_modbus_div_round_up_u64(one_char_numerator * 3ULL,
                                         (uint64_t)baud_rate * 2ULL);
    *t35_us = ld_modbus_div_round_up_u64(one_char_numerator * 7ULL,
                                         (uint64_t)baud_rate * 2ULL);
}

/** @brief Initialize the reusable, buffer-independent RTU timing engine. */
bool ld_modbus_rtu_timing_init(ld_modbus_rtu_timing_t *timing,
                               uint32_t baud_rate,
                               uint8_t bits_per_char,
                               uint32_t timestamp_hz)
{
    uint32_t character_multiplier;

    if((timing == NULL) || (baud_rate == 0U) || (bits_per_char == 0U) ||
       (timestamp_hz == 0U))
    {
        return false;
    }

    memset(timing, 0, sizeof(*timing));
    timing->timestamp_hz = timestamp_hz;
    character_multiplier = bits_per_char;
    timing->char_ticks = ld_modbus_mul_div_round_up_u32(timestamp_hz,
                                                        character_multiplier,
                                                        baud_rate);
    if(baud_rate > LD_MODBUS_RTU_FIXED_GAP_BAUDRATE)
    {
        timing->t15_ticks = ld_modbus_mul_div_round_up_u32(
            timestamp_hz,
            LD_MODBUS_RTU_FIXED_T15_US,
            (uint32_t)LD_MODBUS_US_PER_SECOND);
        timing->t35_ticks = ld_modbus_mul_div_round_up_u32(
            timestamp_hz,
            LD_MODBUS_RTU_FIXED_T35_US,
            (uint32_t)LD_MODBUS_US_PER_SECOND);
    }
    else
    {
        timing->t15_ticks = ld_modbus_mul_div_round_up_u32(
            timestamp_hz,
            character_multiplier * 3U,
            baud_rate * 2U);
        timing->t35_ticks = ld_modbus_mul_div_round_up_u32(
            timestamp_hz,
            character_multiplier * 7U,
            baud_rate * 2U);
    }

    if((timing->char_ticks == 0U) || (timing->t15_ticks == 0U) ||
       (timing->t35_ticks == 0U) ||
       (timing->t15_ticks > LD_MODBUS_WRAP_SAFE_MAX_TICKS) ||
       (timing->t35_ticks > LD_MODBUS_WRAP_SAFE_MAX_TICKS) ||
       (timing->char_ticks > LD_MODBUS_WRAP_SAFE_MAX_TICKS -
                                  timing->t35_ticks))
    {
        memset(timing, 0, sizeof(*timing));
        return false;
    }
    return true;
}

/** @brief Classify one byte before the payload owner stores it. */
ld_modbus_rtu_byte_action_t ld_modbus_rtu_timing_on_byte(
    ld_modbus_rtu_timing_t *timing,
    uint32_t timestamp_ticks)
{
    uint32_t elapsed_ticks;
    uint32_t completion_t15_ticks;
    uint32_t completion_t35_ticks;
    ld_modbus_rtu_byte_action_t action;

    if((timing == NULL) || (timing->timestamp_hz == 0U))
    {
        return LD_MODBUS_RTU_BYTE_DISCARD;
    }

    if(timing->state == LD_MODBUS_RTU_TIMING_IDLE)
    {
        timing->last_byte_ticks = timestamp_ticks;
        timing->state = LD_MODBUS_RTU_TIMING_RECEIVING;
        return LD_MODBUS_RTU_BYTE_ACCEPT;
    }

    elapsed_ticks = timestamp_ticks - timing->last_byte_ticks;
    if(elapsed_ticks > LD_MODBUS_WRAP_SAFE_MAX_TICKS)
    {
        timing->last_byte_ticks = timestamp_ticks;
        timing->state = LD_MODBUS_RTU_TIMING_DISCARDING;
        return LD_MODBUS_RTU_BYTE_ABORT_BAD_TIMESTAMP;
    }
    completion_t15_ticks = timing->char_ticks + timing->t15_ticks;
    completion_t35_ticks = timing->char_ticks + timing->t35_ticks;

    if(timing->state == LD_MODBUS_RTU_TIMING_DISCARDING)
    {
        if(elapsed_ticks >= completion_t35_ticks)
        {
            timing->state = LD_MODBUS_RTU_TIMING_RECEIVING;
            action = LD_MODBUS_RTU_BYTE_ACCEPT;
        }
        else
        {
            action = LD_MODBUS_RTU_BYTE_DISCARD;
        }
        timing->last_byte_ticks = timestamp_ticks;
        return action;
    }

    if(elapsed_ticks >= completion_t35_ticks)
    {
        action = LD_MODBUS_RTU_BYTE_COMMIT_AND_ACCEPT;
    }
    else if(elapsed_ticks > completion_t15_ticks)
    {
        timing->state = LD_MODBUS_RTU_TIMING_DISCARDING;
        action = LD_MODBUS_RTU_BYTE_ABORT_AND_DISCARD;
    }
    else
    {
        action = LD_MODBUS_RTU_BYTE_ACCEPT;
    }
    timing->last_byte_ticks = timestamp_ticks;
    return action;
}

/** @brief Invalidate a stream when the platform reports receive data loss. */
void ld_modbus_rtu_timing_on_error(ld_modbus_rtu_timing_t *timing,
                                   uint32_t timestamp_ticks)
{
    if((timing == NULL) || (timing->timestamp_hz == 0U))
    {
        return;
    }
    timing->last_byte_ticks = timestamp_ticks;
    timing->state = LD_MODBUS_RTU_TIMING_DISCARDING;
}

/** @brief Classify a caller-observed silent interval. */
ld_modbus_rtu_poll_action_t ld_modbus_rtu_timing_poll(
    ld_modbus_rtu_timing_t *timing,
    uint32_t now_ticks)
{
    uint32_t elapsed_ticks;

    if((timing == NULL) || (timing->timestamp_hz == 0U) ||
       (timing->state == LD_MODBUS_RTU_TIMING_IDLE))
    {
        return LD_MODBUS_RTU_POLL_NONE;
    }
    elapsed_ticks = now_ticks - timing->last_byte_ticks;
    if(elapsed_ticks > LD_MODBUS_WRAP_SAFE_MAX_TICKS)
    {
        return LD_MODBUS_RTU_POLL_NONE;
    }

    /* With end-of-character timestamps, wait one additional character time.
     * Any character that started before the T3.5 boundary must be allowed to
     * complete and reach on_byte(), where its true inter-character silence
     * can be classified. */
    if(elapsed_ticks < (timing->char_ticks + timing->t35_ticks))
    {
        return LD_MODBUS_RTU_POLL_NONE;
    }

    if(timing->state == LD_MODBUS_RTU_TIMING_DISCARDING)
    {
        timing->state = LD_MODBUS_RTU_TIMING_IDLE;
        return LD_MODBUS_RTU_POLL_RECOVER;
    }

    timing->state = LD_MODBUS_RTU_TIMING_IDLE;
    return LD_MODBUS_RTU_POLL_COMMIT;
}

/** @brief Invalidate the current stream until a legal T3.5 boundary. */
void ld_modbus_rtu_timing_force_discard(ld_modbus_rtu_timing_t *timing)
{
    if((timing != NULL) &&
       (timing->state == LD_MODBUS_RTU_TIMING_RECEIVING))
    {
        timing->state = LD_MODBUS_RTU_TIMING_DISCARDING;
    }
}

/** @brief Reset receive state while retaining calculated thresholds. */
void ld_modbus_rtu_timing_reset(ld_modbus_rtu_timing_t *timing)
{
    if(timing != NULL)
    {
        timing->last_byte_ticks = 0U;
        timing->state = LD_MODBUS_RTU_TIMING_IDLE;
    }
}

/** @brief Initialize a framer with two caller-owned buffers of equal capacity. */
bool ld_modbus_rtu_framer_init(ld_modbus_rtu_framer_t *ctx,
                               uint8_t *active_buffer,
                               uint8_t *ready_buffer,
                               uint16_t capacity,
                               uint32_t baud_rate,
                               uint8_t bits_per_char,
                               uint32_t timestamp_hz)
{
    ld_modbus_rtu_timing_t timing;

    if((ctx == NULL) ||
       (capacity > LD_MODBUS_RTU_MAX_ADU_LENGTH) ||
       !ld_modbus_rtu_regions_are_disjoint(active_buffer,
                                           capacity,
                                           ready_buffer,
                                           capacity) ||
       !ld_modbus_rtu_regions_are_disjoint(ctx,
                                           sizeof(*ctx),
                                           active_buffer,
                                           capacity) ||
       !ld_modbus_rtu_regions_are_disjoint(ctx,
                                           sizeof(*ctx),
                                           ready_buffer,
                                           capacity) ||
       !ld_modbus_rtu_timing_init(&timing,
                                  baud_rate,
                                  bits_per_char,
                                  timestamp_hz))
    {
        return false;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->active_buffer = active_buffer;
    ctx->ready_buffer = ready_buffer;
    ctx->capacity = capacity;
    ctx->timing = timing;
    return true;
}

/** @brief Replace timing after aborting only the partial receive stream. */
bool ld_modbus_rtu_framer_reconfigure(ld_modbus_rtu_framer_t *ctx,
                                      uint32_t baud_rate,
                                      uint8_t bits_per_char,
                                      uint32_t timestamp_hz)
{
    ld_modbus_rtu_timing_t replacement;

    if((ctx == NULL) || (ctx->active_buffer == NULL) ||
       !ld_modbus_rtu_timing_init(&replacement,
                                  baud_rate,
                                  bits_per_char,
                                  timestamp_hz))
    {
        return false;
    }
    ctx->active_length = 0U;
    ctx->active_open = false;
    ctx->timing = replacement;
    return true;
}

/** @brief Feed one byte and its end-of-character timestamp. */
void ld_modbus_rtu_framer_on_byte(ld_modbus_rtu_framer_t *ctx,
                                  uint8_t byte,
                                  uint32_t timestamp_ticks)
{
    ld_modbus_rtu_byte_action_t action;

    if((ctx == NULL) || (ctx->active_buffer == NULL))
    {
        return;
    }

    action = ld_modbus_rtu_timing_on_byte(&ctx->timing, timestamp_ticks);
    if(action == LD_MODBUS_RTU_BYTE_DISCARD)
    {
        ctx->diag.discarded_bytes++;
        return;
    }
    if(action == LD_MODBUS_RTU_BYTE_ABORT_AND_DISCARD)
    {
        ctx->diag.t15_violations++;
        ctx->diag.discarded_bytes += (uint32_t)ctx->active_length + 1U;
        ctx->active_length = 0U;
        ctx->active_open = false;
        return;
    }
    if(action == LD_MODBUS_RTU_BYTE_ABORT_BAD_TIMESTAMP)
    {
        ctx->diag.timestamp_errors++;
        ctx->diag.discarded_bytes += (uint32_t)ctx->active_length + 1U;
        ctx->active_length = 0U;
        ctx->active_open = false;
        return;
    }
    if(action == LD_MODBUS_RTU_BYTE_COMMIT_AND_ACCEPT)
    {
        ld_modbus_rtu_framer_commit_active(ctx);
    }

    if(ctx->active_length >= ctx->capacity)
    {
        ctx->diag.overflow++;
        ctx->diag.discarded_bytes += (uint32_t)ctx->active_length + 1U;
        ctx->active_length = 0U;
        ctx->active_open = false;
        ld_modbus_rtu_timing_force_discard(&ctx->timing);
        return;
    }

    ctx->active_buffer[ctx->active_length++] = byte;
    ctx->active_open = true;
}

/** @brief Abort a partial frame after a platform receive-error event. */
void ld_modbus_rtu_framer_on_error(ld_modbus_rtu_framer_t *ctx,
                                   uint32_t timestamp_ticks)
{
    if((ctx == NULL) || (ctx->active_buffer == NULL) ||
       (ctx->timing.timestamp_hz == 0U))
    {
        return;
    }
    ctx->diag.rx_errors++;
    ctx->diag.discarded_bytes += ctx->active_length;
    ctx->active_length = 0U;
    ctx->active_open = false;
    ld_modbus_rtu_timing_on_error(&ctx->timing, timestamp_ticks);
}

/** @brief Commit an open frame after T3.5 silence. */
void ld_modbus_rtu_framer_poll(ld_modbus_rtu_framer_t *ctx,
                               uint32_t now_ticks)
{
    if(ctx == NULL)
    {
        return;
    }

    if(ld_modbus_rtu_timing_poll(&ctx->timing, now_ticks) ==
       LD_MODBUS_RTU_POLL_COMMIT)
    {
        ld_modbus_rtu_framer_commit_active(ctx);
    }
}

/** @brief Claim one stable completed frame without copying payload bytes. */
bool ld_modbus_rtu_framer_claim(ld_modbus_rtu_framer_t *ctx,
                                ld_modbus_rtu_frame_view_t *view)
{
    if((ctx == NULL) || (view == NULL) ||
       !ld_modbus_rtu_regions_are_disjoint(view,
                                           sizeof(*view),
                                           ctx,
                                           sizeof(*ctx)) ||
       !ld_modbus_rtu_regions_are_disjoint(view,
                                           sizeof(*view),
                                           ctx->active_buffer,
                                           ctx->capacity) ||
       !ld_modbus_rtu_regions_are_disjoint(view,
                                           sizeof(*view),
                                           ctx->ready_buffer,
                                           ctx->capacity))
    {
        return false;
    }
    memset(view, 0, sizeof(*view));
    if(!ctx->ready_available || ctx->ready_claimed)
    {
        return false;
    }
    ctx->ready_claimed = true;
    view->data = ctx->ready_buffer;
    view->length = ctx->ready_length;
    view->generation = ctx->ready_generation;
    return true;
}

/** @brief Consume and release a previously claimed frame view. */
bool ld_modbus_rtu_framer_release(ld_modbus_rtu_framer_t *ctx,
                                  ld_modbus_rtu_frame_view_t *view)
{
    if((ctx == NULL) || (view == NULL) ||
       !ld_modbus_rtu_regions_are_disjoint(view,
                                           sizeof(*view),
                                           ctx,
                                           sizeof(*ctx)) ||
       !ld_modbus_rtu_regions_are_disjoint(view,
                                           sizeof(*view),
                                           ctx->active_buffer,
                                           ctx->capacity) ||
       !ld_modbus_rtu_regions_are_disjoint(view,
                                           sizeof(*view),
                                           ctx->ready_buffer,
                                           ctx->capacity) ||
       !ctx->ready_available ||
       !ctx->ready_claimed || (view->data != ctx->ready_buffer) ||
       (view->length != ctx->ready_length) ||
       (view->generation != ctx->ready_generation))
    {
        return false;
    }
    ctx->ready_length = 0U;
    ctx->ready_available = false;
    ctx->ready_claimed = false;
    memset(view, 0, sizeof(*view));
    return true;
}

/** @brief Copy and consume one completed frame. */
bool ld_modbus_rtu_framer_take(ld_modbus_rtu_framer_t *ctx,
                               uint8_t *out,
                               uint16_t out_capacity,
                               uint16_t *out_length)
{
    ld_modbus_rtu_frame_view_t view;
    uint16_t frame_length;

    if((ctx == NULL) || (out == NULL) || (out_length == NULL) ||
       !ld_modbus_rtu_regions_are_disjoint(out_length,
                                           sizeof(*out_length),
                                           ctx,
                                           sizeof(*ctx)) ||
       !ld_modbus_rtu_regions_are_disjoint(out_length,
                                           sizeof(*out_length),
                                           ctx->active_buffer,
                                           ctx->capacity) ||
       !ld_modbus_rtu_regions_are_disjoint(out_length,
                                           sizeof(*out_length),
                                           ctx->ready_buffer,
                                           ctx->capacity) ||
       ((out_capacity != 0U) &&
        (!ld_modbus_rtu_regions_are_disjoint(out,
                                             out_capacity,
                                             ctx,
                                             sizeof(*ctx)) ||
         !ld_modbus_rtu_regions_are_disjoint(out,
                                             out_capacity,
                                             ctx->active_buffer,
                                             ctx->capacity) ||
         !ld_modbus_rtu_regions_are_disjoint(out,
                                             out_capacity,
                                             ctx->ready_buffer,
                                             ctx->capacity) ||
         !ld_modbus_rtu_regions_are_disjoint(out,
                                             out_capacity,
                                             out_length,
                                             sizeof(*out_length)))))
    {
        return false;
    }

    *out_length = 0U;
    if(!ld_modbus_rtu_framer_claim(ctx, &view))
    {
        return false;
    }
    if(out_capacity < view.length)
    {
        ctx->ready_claimed = false;
        return false;
    }

    frame_length = view.length;
    memcpy(out, view.data, frame_length);
    if(!ld_modbus_rtu_framer_release(ctx, &view))
    {
        return false;
    }
    *out_length = frame_length;
    return true;
}

/** @brief Drop open and completed data while retaining diagnostics and timing. */
bool ld_modbus_rtu_framer_reset(ld_modbus_rtu_framer_t *ctx)
{
    if((ctx == NULL) || ctx->ready_claimed)
    {
        return false;
    }
    ctx->active_length = 0U;
    ctx->ready_length = 0U;
    ctx->active_open = false;
    ctx->ready_available = false;
    ld_modbus_rtu_timing_reset(&ctx->timing);
    return true;
}
