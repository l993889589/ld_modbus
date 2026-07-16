/**
 * @file test_ld_modbus_rtu_receiver.c
 * @brief End-to-end tests for the dependency-free RTU receiver and server.
 */

#include "ld_modbus_client.h"
#include "ld_modbus_rtu_framer.h"
#include "ld_modbus_server.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static ld_modbus_rtu_framer_t test_receiver;
static uint8_t test_active[LD_MODBUS_RTU_MAX_ADU_LENGTH];
static uint8_t test_ready[LD_MODBUS_RTU_MAX_ADU_LENGTH];
static uint32_t next_timestamp;

/** @brief Feed one complete continuous ADU and commit it after T3.5. */
static void test_push_frame(const uint8_t *frame, size_t length)
{
    size_t index;

    for(index = 0U; index < length; index++)
    {
        ld_modbus_rtu_framer_on_byte(&test_receiver,
                                      frame[index],
                                      next_timestamp);
        next_timestamp += test_receiver.timing.char_ticks;
    }
    ld_modbus_rtu_framer_poll(
        &test_receiver,
        test_receiver.timing.last_byte_ticks +
            test_receiver.timing.char_ticks +
            test_receiver.timing.t35_ticks);
    next_timestamp = test_receiver.timing.last_byte_ticks +
                     test_receiver.timing.t35_ticks +
                     test_receiver.timing.char_ticks;
}

/** @brief Verify an illegal T1.5..T3.5 gap publishes no tail fragment. */
static void test_t15_violation_discards_stream(const uint8_t *frame,
                                               size_t length)
{
    ld_modbus_rtu_frame_view_t view;
    uint32_t frames_before = test_receiver.diag.frames_completed;
    size_t index;

    assert(length > 3U);
    ld_modbus_rtu_framer_on_byte(&test_receiver,
                                  frame[0],
                                  next_timestamp);
    next_timestamp += test_receiver.timing.char_ticks;
    ld_modbus_rtu_framer_on_byte(&test_receiver,
                                  frame[1],
                                  next_timestamp);
    next_timestamp += test_receiver.timing.char_ticks +
                      test_receiver.timing.t15_ticks + 1U;
    ld_modbus_rtu_framer_on_byte(&test_receiver,
                                  frame[2],
                                  next_timestamp);
    for(index = 3U; index < length; index++)
    {
        next_timestamp += test_receiver.timing.char_ticks;
        ld_modbus_rtu_framer_on_byte(&test_receiver,
                                      frame[index],
                                      next_timestamp);
    }
    ld_modbus_rtu_framer_poll(
        &test_receiver,
        test_receiver.timing.last_byte_ticks +
            test_receiver.timing.char_ticks +
            test_receiver.timing.t35_ticks);
    assert(test_receiver.diag.frames_completed == frames_before);
    assert(test_receiver.diag.t15_violations == 1U);
    assert(!ld_modbus_rtu_framer_claim(&test_receiver, &view));
    next_timestamp = test_receiver.timing.last_byte_ticks +
                     test_receiver.timing.t35_ticks +
                     test_receiver.timing.char_ticks;
}

/** @brief Run dependency-free RTU receive and server integration tests. */
int main(void)
{
    ld_modbus_server_map_t map;
    ld_modbus_server_action_t action;
    ld_modbus_rtu_frame_view_t request;
    ld_modbus_adu_view_t response_view;
    uint16_t holding[16];
    uint16_t parsed[2];
    uint8_t pdu[LD_MODBUS_MAX_PDU_LENGTH];
    uint8_t request_adu[LD_MODBUS_RTU_MAX_ADU_LENGTH];
    uint8_t response_adu[LD_MODBUS_RTU_MAX_ADU_LENGTH];
    size_t pdu_length;
    size_t request_length;
    size_t response_length;
    uint16_t index;

    assert(ld_modbus_rtu_framer_init(&test_receiver,
                                      test_active,
                                      test_ready,
                                      sizeof(test_active),
                                      9600U,
                                      10U,
                                      1000000U));
    next_timestamp = 1000U;
    memset(&map, 0, sizeof(map));
    for(index = 0U; index < 16U; index++)
    {
        holding[index] = (uint16_t)(0x1000U + index);
    }
    map.holding_registers = holding;
    map.holding_registers_count = 16U;

    assert(ld_modbus_client_build_read_request(
               LD_MODBUS_FC_READ_HOLDING_REGISTERS,
               2U,
               2U,
               pdu,
               sizeof(pdu),
               &pdu_length) == LD_MODBUS_STATUS_OK);
    assert(ld_modbus_rtu_encode(1U,
                                pdu,
                                pdu_length,
                                request_adu,
                                sizeof(request_adu),
                                &request_length) == LD_MODBUS_STATUS_OK);
    test_t15_violation_discards_stream(request_adu, request_length);

    test_push_frame(request_adu, request_length);
    assert(ld_modbus_rtu_framer_claim(&test_receiver, &request));
    assert(ld_modbus_server_process_rtu_adu(&map,
                                             1U,
                                             request.data,
                                             request.length,
                                             response_adu,
                                             sizeof(response_adu),
                                             &response_length,
                                             &action) ==
           LD_MODBUS_STATUS_OK);
    assert(action == LD_MODBUS_SERVER_ACTION_REPLY);
    assert(ld_modbus_rtu_framer_release(&test_receiver, &request));
    assert(ld_modbus_rtu_decode(response_adu,
                                response_length,
                                &response_view) == LD_MODBUS_STATUS_OK);
    assert(ld_modbus_client_parse_read_registers_response(
               LD_MODBUS_FC_READ_HOLDING_REGISTERS,
               2U,
               response_view.pdu,
               response_view.pdu_length,
               parsed,
               2U,
               NULL) == LD_MODBUS_STATUS_OK);
    assert(parsed[0] == 0x1002U && parsed[1] == 0x1003U);

    request_adu[request_length - 1U] ^= 1U;
    test_push_frame(request_adu, request_length);
    assert(ld_modbus_rtu_framer_claim(&test_receiver, &request));
    assert(ld_modbus_server_process_rtu_adu(&map,
                                             1U,
                                             request.data,
                                             request.length,
                                             response_adu,
                                             sizeof(response_adu),
                                             &response_length,
                                             &action) ==
           LD_MODBUS_STATUS_BAD_CRC);
    assert(ld_modbus_rtu_framer_release(&test_receiver, &request));

    puts("ld_modbus dependency-free RTU receiver tests passed");
    return 0;
}
