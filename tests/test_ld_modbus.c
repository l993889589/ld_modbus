/**
 * @file test_ld_modbus.c
 * @brief Host tests for bounded ld_modbus codecs and PDU processing.
 */

#include "ld_modbus_client.h"
#include "ld_modbus_server.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

/** @brief Verify the standard CRC vector and RTU codec round trip. */
static void test_rtu_codec(void)
{
    static const uint8_t expected[] = {0x01U, 0x03U, 0x00U, 0x00U,
                                       0x00U, 0x0AU, 0xC5U, 0xCDU};
    uint8_t pdu[] = {0x03U, 0x00U, 0x00U, 0x00U, 0x0AU};
    uint8_t adu[LD_MODBUS_RTU_MAX_ADU_LENGTH];
    ld_modbus_adu_view_t view;
    size_t length = 0U;

    assert(ld_modbus_rtu_encode(1U, pdu, sizeof(pdu), adu, sizeof(adu), &length) ==
           LD_MODBUS_STATUS_OK);
    assert(length == sizeof(expected));
    assert(memcmp(adu, expected, sizeof(expected)) == 0);
    assert(ld_modbus_rtu_decode(adu, length, &view) == LD_MODBUS_STATUS_OK);
    assert(view.unit_id == 1U && view.pdu_length == sizeof(pdu));
    assert(memcmp(view.pdu, pdu, sizeof(pdu)) == 0);
    adu[3] ^= 1U;
    assert(ld_modbus_rtu_decode(adu, length, &view) == LD_MODBUS_STATUS_BAD_CRC);
}

/** @brief Verify MBAP lengths, transaction identifiers, and protocol checks. */
static void test_tcp_codec(void)
{
    uint8_t pdu[] = {0x06U, 0x00U, 0x02U, 0x12U, 0x34U};
    uint8_t adu[LD_MODBUS_TCP_MAX_ADU_LENGTH];
    ld_modbus_adu_view_t view;
    size_t length = 0U;

    assert(ld_modbus_tcp_encode(0x1234U, 7U, pdu, sizeof(pdu), adu, sizeof(adu), &length) ==
           LD_MODBUS_STATUS_OK);
    assert(length == 12U && adu[4] == 0U && adu[5] == 6U);
    assert(ld_modbus_tcp_decode(adu, length, &view) == LD_MODBUS_STATUS_OK);
    assert(view.transaction_id == 0x1234U && view.unit_id == 7U);
    adu[3] = 1U;
    assert(ld_modbus_tcp_decode(adu, length, &view) == LD_MODBUS_STATUS_BAD_PROTOCOL_ID);
}

/** @brief Verify static in-place RTU and TCP framing preserves the PDU. */
static void test_in_place_codec(void)
{
    uint8_t buffer[LD_MODBUS_TCP_MAX_ADU_LENGTH] = {0x03U, 0x00U, 0x02U,
                                                     0x00U, 0x01U};
    ld_modbus_adu_view_t view;
    size_t length = 0U;

    assert(ld_modbus_rtu_encode(1U, buffer, 5U, buffer, sizeof(buffer), &length) ==
           LD_MODBUS_STATUS_OK);
    assert(ld_modbus_rtu_decode(buffer, length, &view) == LD_MODBUS_STATUS_OK);
    assert(view.pdu[0] == LD_MODBUS_FC_READ_HOLDING_REGISTERS);

    memcpy(buffer, (uint8_t[]){0x06U, 0x00U, 0x01U, 0x12U, 0x34U}, 5U);
    assert(ld_modbus_tcp_encode(0x55AAU, 1U, buffer, 5U,
                                buffer, sizeof(buffer), &length) == LD_MODBUS_STATUS_OK);
    assert(ld_modbus_tcp_decode(buffer, length, &view) == LD_MODBUS_STATUS_OK);
    assert(view.transaction_id == 0x55AAU &&
           view.pdu[0] == LD_MODBUS_FC_WRITE_SINGLE_REGISTER);
}

/** @brief Verify client builders and strict response-length validation. */
static void test_client_helpers(void)
{
    uint8_t pdu[LD_MODBUS_MAX_PDU_LENGTH];
    uint16_t registers[3];
    const uint16_t write_values[] = {0x1234U, 0xABCDU};
    uint8_t exception = 0U;
    size_t length = 0U;
    const uint8_t response[] = {0x03U, 0x06U, 0x00U, 0x01U,
                                0x12U, 0x34U, 0xABU, 0xCDU};

    assert(ld_modbus_client_build_read_request(LD_MODBUS_FC_READ_HOLDING_REGISTERS,
                                               4U, 3U, pdu, sizeof(pdu), &length) ==
           LD_MODBUS_STATUS_OK);
    assert(length == 5U && pdu[0] == 0x03U && pdu[4] == 3U);
    assert(ld_modbus_client_parse_read_registers_response(0x03U, 3U,
                                                          response, sizeof(response),
                                                          registers, 3U, &exception) ==
           LD_MODBUS_STATUS_OK);
    assert(registers[0] == 1U && registers[1] == 0x1234U && registers[2] == 0xABCDU);
    assert(ld_modbus_client_parse_read_registers_response(0x03U, 2U,
                                                          response, sizeof(response),
                                                          registers, 3U, &exception) ==
           LD_MODBUS_STATUS_MALFORMED_FRAME);

    assert(ld_modbus_client_build_mask_write_register(2U, 0x0F0FU, 0x5050U,
                                                       pdu, sizeof(pdu), &length) ==
           LD_MODBUS_STATUS_OK);
    assert(length == 7U && pdu[0] == LD_MODBUS_FC_MASK_WRITE_REGISTER);
    assert(ld_modbus_client_parse_mask_write_response(2U, 0x0F0FU, 0x5050U,
                                                       pdu, length, &exception) ==
           LD_MODBUS_STATUS_OK);

    assert(ld_modbus_client_build_write_read_multiple_registers(
               1U, 3U, 5U, write_values, 2U,
               pdu, sizeof(pdu), &length) == LD_MODBUS_STATUS_OK);
    assert(length == 14U && pdu[0] == LD_MODBUS_FC_WRITE_READ_MULTIPLE_REGISTERS &&
           pdu[9] == 4U && pdu[10] == 0x12U && pdu[13] == 0xCDU);
}

/** @brief Verify reads, atomic range rejection, writes, and exception PDUs. */
static void test_server_map(void)
{
    uint8_t coils[16] = {0U};
    uint8_t inputs[16] = {0U};
    uint16_t holding[16];
    uint16_t input_registers[16];
    ld_modbus_server_map_t map;
    uint8_t response[LD_MODBUS_MAX_PDU_LENGTH];
    size_t response_length = 0U;
    uint16_t index;

    memset(&map, 0, sizeof(map));
    for(index = 0U; index < 16U; ++index)
    {
        holding[index] = index;
        input_registers[index] = (uint16_t)(100U + index);
    }
    map.coils = coils;
    map.coils_count = 16U;
    map.discrete_inputs = inputs;
    map.discrete_inputs_count = 16U;
    map.holding_registers = holding;
    map.holding_registers_count = 16U;
    map.input_registers = input_registers;
    map.input_registers_count = 16U;

    {
        const uint8_t request[] = {0x03U, 0x00U, 0x01U, 0x00U, 0x03U};
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(response_length == 8U && response[0] == 0x03U && response[1] == 6U);
        assert(response[3] == 1U && response[5] == 2U && response[7] == 3U);
    }
    {
        const uint8_t request[] = {0x06U, 0x00U, 0x02U, 0x12U, 0x34U};
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(holding[2] == 0x1234U && memcmp(response, request, sizeof(request)) == 0);
    }
    {
        const uint8_t request[] = {0x10U, 0x00U, 0x0FU, 0x00U, 0x02U,
                                   0x04U, 0x11U, 0x11U, 0x22U, 0x22U};
        uint16_t original = holding[15];
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(response_length == 2U && response[0] == 0x90U &&
               response[1] == LD_MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        assert(holding[15] == original);
    }
    {
        const uint8_t request[] = {0x55U};
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(response_length == 2U && response[0] == 0xD5U &&
               response[1] == LD_MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
    }
}

/** @brief Verify application-side accessors for all four Modbus data tables. */
static void test_server_map_accessors(void)
{
    uint8_t coils[3] = {0U, 0U, 0U};
    uint8_t discrete[2] = {0U, 0U};
    uint16_t holding[3] = {0U, 0U, 0U};
    uint16_t input[2] = {0U, 0U};
    ld_modbus_server_map_t map;
    uint8_t bit_value = 0U;
    uint16_t register_value = 0U;

    memset(&map, 0, sizeof(map));
    map.coils = coils;
    map.coils_start = 10U;
    map.coils_count = 3U;
    map.discrete_inputs = discrete;
    map.discrete_inputs_start = 20U;
    map.discrete_inputs_count = 2U;
    map.holding_registers = holding;
    map.holding_registers_start = 100U;
    map.holding_registers_count = 3U;
    map.input_registers = input;
    map.input_registers_start = 200U;
    map.input_registers_count = 2U;

    assert(ld_modbus_server_map_write_coil(&map, 11U, 9U) == LD_MODBUS_STATUS_OK);
    assert(coils[1] == 1U);
    assert(ld_modbus_server_map_read_coil(&map, 11U, &bit_value) ==
           LD_MODBUS_STATUS_OK);
    assert(bit_value == 1U);

    assert(ld_modbus_server_map_set_discrete_input(&map, 20U, 2U) ==
           LD_MODBUS_STATUS_OK);
    assert(ld_modbus_server_map_read_discrete_input(&map, 20U, &bit_value) ==
           LD_MODBUS_STATUS_OK);
    assert(bit_value == 1U && discrete[0] == 1U);

    assert(ld_modbus_server_map_write_holding_register(&map, 102U, 0x1234U) ==
           LD_MODBUS_STATUS_OK);
    assert(ld_modbus_server_map_read_holding_register(&map, 102U, &register_value) ==
           LD_MODBUS_STATUS_OK);
    assert(register_value == 0x1234U);

    assert(ld_modbus_server_map_set_input_register(&map, 201U, 0xABCDU) ==
           LD_MODBUS_STATUS_OK);
    assert(ld_modbus_server_map_read_input_register(&map, 201U, &register_value) ==
           LD_MODBUS_STATUS_OK);
    assert(register_value == 0xABCDU);

    assert(ld_modbus_server_map_read_coil(NULL, 10U, &bit_value) ==
           LD_MODBUS_STATUS_INVALID_ARGUMENT);
    assert(ld_modbus_server_map_read_coil(&map, 10U, NULL) ==
           LD_MODBUS_STATUS_INVALID_ARGUMENT);
    assert(ld_modbus_server_map_read_coil(&map, 9U, &bit_value) ==
           LD_MODBUS_STATUS_RANGE_ERROR);
    assert(ld_modbus_server_map_write_coil(&map, 13U, 1U) ==
           LD_MODBUS_STATUS_RANGE_ERROR);
    assert(ld_modbus_server_map_read_holding_register(&map, 103U, &register_value) ==
           LD_MODBUS_STATUS_RANGE_ERROR);

    map.input_registers = NULL;
    assert(ld_modbus_server_map_set_input_register(&map, 200U, 1U) ==
           LD_MODBUS_STATUS_RANGE_ERROR);
}

/** @brief Exercise every v0.1 function code and both address/value exceptions. */
static void test_all_function_codes(void)
{
    uint8_t coils[16] = {1U, 0U, 1U, 0U};
    uint8_t discrete[16] = {1U, 0U, 1U, 1U};
    uint16_t holding[16] = {0U};
    uint16_t input[16] = {0x1111U, 0x2222U};
    ld_modbus_server_map_t map;
    uint8_t response[LD_MODBUS_MAX_PDU_LENGTH];
    uint8_t bits[4];
    uint16_t registers[2];
    uint8_t exception = 0U;
    size_t response_length;

    memset(&map, 0, sizeof(map));
    map.coils = coils;
    map.coils_count = 16U;
    map.discrete_inputs = discrete;
    map.discrete_inputs_count = 16U;
    map.holding_registers = holding;
    map.holding_registers_count = 16U;
    map.input_registers = input;
    map.input_registers_count = 16U;

    {
        const uint8_t request[] = {0x01U, 0U, 0U, 0U, 4U};
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(ld_modbus_client_parse_read_bits_response(
                   LD_MODBUS_FC_READ_COILS, 4U, response, response_length,
                   bits, 4U, &exception) == LD_MODBUS_STATUS_OK);
        assert(bits[0] == 1U && bits[1] == 0U && bits[2] == 1U);
    }
    {
        const uint8_t request[] = {0x02U, 0U, 0U, 0U, 4U};
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(ld_modbus_client_parse_read_bits_response(
                   LD_MODBUS_FC_READ_DISCRETE_INPUTS, 4U, response, response_length,
                   bits, 4U, &exception) == LD_MODBUS_STATUS_OK);
        assert(bits[0] == 1U && bits[2] == 1U && bits[3] == 1U);
    }
    {
        const uint8_t request[] = {0x04U, 0U, 0U, 0U, 2U};
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(ld_modbus_client_parse_read_registers_response(
                   LD_MODBUS_FC_READ_INPUT_REGISTERS, 2U, response, response_length,
                   registers, 2U, &exception) == LD_MODBUS_STATUS_OK);
        assert(registers[0] == 0x1111U && registers[1] == 0x2222U);
    }
    {
        const uint8_t request[] = {0x05U, 0U, 1U, 0xFFU, 0U};
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(coils[1] == 1U && memcmp(request, response, sizeof(request)) == 0);
        assert(ld_modbus_client_parse_write_response(
                   LD_MODBUS_FC_WRITE_SINGLE_COIL, 1U, 0xFF00U,
                   response, response_length, &exception) == LD_MODBUS_STATUS_OK);
    }
    {
        const uint8_t request[] = {0x0FU, 0U, 4U, 0U, 4U, 1U, 0x0DU};
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(coils[4] == 1U && coils[5] == 0U && coils[6] == 1U && coils[7] == 1U);
        assert(ld_modbus_client_parse_write_response(
                   LD_MODBUS_FC_WRITE_MULTIPLE_COILS, 4U, 4U,
                   response, response_length, &exception) == LD_MODBUS_STATUS_OK);
    }
    {
        const uint8_t request[] = {0x10U, 0U, 6U, 0U, 2U, 4U,
                                   0x12U, 0x34U, 0xABU, 0xCDU};
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(holding[6] == 0x1234U && holding[7] == 0xABCDU);
        assert(ld_modbus_client_parse_write_response(
                   LD_MODBUS_FC_WRITE_MULTIPLE_REGISTERS, 6U, 2U,
                   response, response_length, &exception) == LD_MODBUS_STATUS_OK);
    }
    {
        const uint8_t request[] = {0x16U, 0U, 8U, 0x0FU, 0x0FU, 0U, 0x50U};
        holding[8] = 0xAAAAU;
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(holding[8] == 0x0A5AU);
        assert(ld_modbus_client_parse_mask_write_response(
                   8U, 0x0F0FU, 0x0050U, response, response_length,
                   &exception) == LD_MODBUS_STATUS_OK);
    }
    {
        const uint8_t request[] = {0x17U, 0U, 9U, 0U, 2U, 0U, 9U, 0U, 2U,
                                   4U, 0x12U, 0x34U, 0xABU, 0xCDU};
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(ld_modbus_client_parse_read_registers_response(
                   LD_MODBUS_FC_WRITE_READ_MULTIPLE_REGISTERS, 2U,
                   response, response_length, registers, 2U,
                   &exception) == LD_MODBUS_STATUS_OK);
        assert(registers[0] == 0x1234U && registers[1] == 0xABCDU);
    }
    {
        const uint8_t request[] = {0x03U, 0U, 0U, 0U, 0U};
        assert(ld_modbus_server_process_pdu(&map, request, sizeof(request), response,
                                            sizeof(response), &response_length) ==
               LD_MODBUS_STATUS_OK);
        assert(response[0] == 0x83U &&
               response[1] == LD_MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
    }
}

/** @brief Verify complete RTU/TCP server ADUs, routing, and broadcasts. */
static void test_complete_adu_servers(void)
{
    uint16_t holding[8] = {0x1000U, 0x1001U, 0x1002U, 0x1003U,
                           0x1004U, 0x1005U, 0x1006U, 0x1007U};
    ld_modbus_server_map_t map;
    ld_modbus_server_action_t action;
    ld_modbus_adu_view_t view;
    uint8_t request[LD_MODBUS_TCP_MAX_ADU_LENGTH];
    uint8_t response[LD_MODBUS_TCP_MAX_ADU_LENGTH];
    const uint8_t read_pdu[] = {0x03U, 0x00U, 0x01U, 0x00U, 0x02U};
    const uint8_t write_pdu[] = {0x06U, 0x00U, 0x04U, 0xBEU, 0xEFU};
    size_t request_length;
    size_t response_length;

    memset(&map, 0, sizeof(map));
    map.holding_registers = holding;
    map.holding_registers_count = 8U;

    assert(ld_modbus_rtu_encode(1U, read_pdu, sizeof(read_pdu),
                                request, sizeof(request), &request_length) ==
           LD_MODBUS_STATUS_OK);
    assert(ld_modbus_server_process_rtu_adu(&map, 1U,
                                            request, request_length,
                                            response, sizeof(response),
                                            &response_length, &action) ==
           LD_MODBUS_STATUS_OK);
    assert(action == LD_MODBUS_SERVER_ACTION_REPLY && response_length == 9U);
    assert(ld_modbus_rtu_decode(response, response_length, &view) ==
           LD_MODBUS_STATUS_OK);
    assert(view.unit_id == 1U && view.pdu[0] == 0x03U && view.pdu[1] == 4U &&
           view.pdu[3] == 1U && view.pdu[5] == 2U);

    assert(ld_modbus_server_process_rtu_adu(&map, 2U,
                                            request, request_length,
                                            response, sizeof(response),
                                            &response_length, &action) ==
           LD_MODBUS_STATUS_OK);
    assert(action == LD_MODBUS_SERVER_ACTION_IGNORED && response_length == 0U);

    assert(ld_modbus_rtu_encode(0U, write_pdu, sizeof(write_pdu),
                                request, sizeof(request), &request_length) ==
           LD_MODBUS_STATUS_OK);
    assert(ld_modbus_server_process_rtu_adu(&map, 1U,
                                            request, request_length,
                                            response, sizeof(response),
                                            &response_length, &action) ==
           LD_MODBUS_STATUS_OK);
    assert(action == LD_MODBUS_SERVER_ACTION_BROADCAST_APPLIED &&
           response_length == 0U && holding[4] == 0xBEEFU);

    assert(ld_modbus_tcp_encode(0xCAFEU, 7U, read_pdu, sizeof(read_pdu),
                                request, sizeof(request), &request_length) ==
           LD_MODBUS_STATUS_OK);
    assert(ld_modbus_server_process_tcp_adu(&map,
                                            request, request_length,
                                            response, sizeof(response),
                                            &response_length) == LD_MODBUS_STATUS_OK);
    assert(ld_modbus_tcp_decode(response, response_length, &view) ==
           LD_MODBUS_STATUS_OK);
    assert(view.transaction_id == 0xCAFEU && view.unit_id == 7U &&
           view.pdu[0] == 0x03U && view.pdu[1] == 4U);
}

/** @brief Run all host-side core regression cases. */
int main(void)
{
    test_rtu_codec();
    test_tcp_codec();
    test_in_place_codec();
    test_client_helpers();
    test_server_map();
    test_server_map_accessors();
    test_all_function_codes();
    test_complete_adu_servers();
    puts("ld_modbus tests passed");
    return 0;
}
