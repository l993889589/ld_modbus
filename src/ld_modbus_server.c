/**
 * @file ld_modbus_server.c
 * @brief Modbus server PDU engine over caller-owned static mappings.
 */

#include "ld_modbus_server.h"

#include <string.h>

/** @brief Read one big-endian 16-bit value. */
static uint16_t ld_modbus_get_u16(const uint8_t *source)
{
    return (uint16_t)((uint16_t)source[0] << 8U) | source[1];
}

/** @brief Store one big-endian 16-bit value. */
static void ld_modbus_put_u16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value >> 8U);
    destination[1] = (uint8_t)value;
}

/** @brief Return nonzero for functions accepted as RTU broadcasts. */
static int ld_modbus_server_is_broadcast_write(uint8_t function)
{
    return function == LD_MODBUS_FC_WRITE_SINGLE_COIL ||
           function == LD_MODBUS_FC_WRITE_SINGLE_REGISTER ||
           function == LD_MODBUS_FC_WRITE_MULTIPLE_COILS ||
           function == LD_MODBUS_FC_WRITE_MULTIPLE_REGISTERS ||
           function == LD_MODBUS_FC_MASK_WRITE_REGISTER ||
           function == LD_MODBUS_FC_WRITE_READ_MULTIPLE_REGISTERS;
}

/** @brief Build a two-byte Modbus exception PDU. */
static ld_modbus_status_t ld_modbus_server_exception(uint8_t function,
                                                     uint8_t exception,
                                                     uint8_t *response,
                                                     size_t response_capacity,
                                                     size_t *response_length)
{
    if(response_capacity < 2U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;
    response[0] = (uint8_t)(function | 0x80U);
    response[1] = exception;
    *response_length = 2U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Validate a request range without 16-bit address wraparound. */
static int ld_modbus_server_range_is_valid(uint16_t address,
                                           uint16_t quantity,
                                           uint16_t start,
                                           uint16_t count)
{
    uint32_t offset;

    if(quantity == 0U || address < start)
        return 0;
    offset = (uint32_t)address - start;
    return offset + quantity <= count;
}

/** @brief Translate one configured Modbus data address into a table index. */
static ld_modbus_status_t ld_modbus_server_map_index(uint16_t address,
                                                     uint16_t start,
                                                     uint16_t count,
                                                     uint16_t *index)
{
    uint32_t offset;

    if(index == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(address < start)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    offset = (uint32_t)address - start;
    if(offset >= count)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    *index = (uint16_t)offset;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Read one normalized coil value through the public application map API. */
ld_modbus_status_t ld_modbus_server_map_read_coil(const ld_modbus_server_map_t *map,
                                                  uint16_t address,
                                                  uint8_t *value)
{
    uint16_t index;
    ld_modbus_status_t status;

    if(map == NULL || value == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(map->coils == NULL)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    status = ld_modbus_server_map_index(address, map->coils_start,
                                        map->coils_count, &index);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    *value = map->coils[index] != 0U ? 1U : 0U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Write one normalized coil value through the public application map API. */
ld_modbus_status_t ld_modbus_server_map_write_coil(ld_modbus_server_map_t *map,
                                                   uint16_t address,
                                                   uint8_t value)
{
    uint16_t index;
    ld_modbus_status_t status;

    if(map == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(map->coils == NULL)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    status = ld_modbus_server_map_index(address, map->coils_start,
                                        map->coils_count, &index);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    map->coils[index] = value != 0U ? 1U : 0U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Read one normalized discrete input through the public application map API. */
ld_modbus_status_t ld_modbus_server_map_read_discrete_input(
    const ld_modbus_server_map_t *map,
    uint16_t address,
    uint8_t *value)
{
    uint16_t index;
    ld_modbus_status_t status;

    if(map == NULL || value == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(map->discrete_inputs == NULL)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    status = ld_modbus_server_map_index(address, map->discrete_inputs_start,
                                        map->discrete_inputs_count, &index);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    *value = map->discrete_inputs[index] != 0U ? 1U : 0U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Update one normalized discrete input from the local application. */
ld_modbus_status_t ld_modbus_server_map_set_discrete_input(ld_modbus_server_map_t *map,
                                                           uint16_t address,
                                                           uint8_t value)
{
    uint16_t index;
    ld_modbus_status_t status;

    if(map == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(map->discrete_inputs == NULL)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    status = ld_modbus_server_map_index(address, map->discrete_inputs_start,
                                        map->discrete_inputs_count, &index);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    map->discrete_inputs[index] = value != 0U ? 1U : 0U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Read one holding register through the public application map API. */
ld_modbus_status_t ld_modbus_server_map_read_holding_register(
    const ld_modbus_server_map_t *map,
    uint16_t address,
    uint16_t *value)
{
    uint16_t index;
    ld_modbus_status_t status;

    if(map == NULL || value == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(map->holding_registers == NULL)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    status = ld_modbus_server_map_index(address, map->holding_registers_start,
                                        map->holding_registers_count, &index);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    *value = map->holding_registers[index];
    return LD_MODBUS_STATUS_OK;
}

/** @brief Write one holding register through the public application map API. */
ld_modbus_status_t ld_modbus_server_map_write_holding_register(
    ld_modbus_server_map_t *map,
    uint16_t address,
    uint16_t value)
{
    uint16_t index;
    ld_modbus_status_t status;

    if(map == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(map->holding_registers == NULL)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    status = ld_modbus_server_map_index(address, map->holding_registers_start,
                                        map->holding_registers_count, &index);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    map->holding_registers[index] = value;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Read one input register through the public application map API. */
ld_modbus_status_t ld_modbus_server_map_read_input_register(
    const ld_modbus_server_map_t *map,
    uint16_t address,
    uint16_t *value)
{
    uint16_t index;
    ld_modbus_status_t status;

    if(map == NULL || value == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(map->input_registers == NULL)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    status = ld_modbus_server_map_index(address, map->input_registers_start,
                                        map->input_registers_count, &index);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    *value = map->input_registers[index];
    return LD_MODBUS_STATUS_OK;
}

/** @brief Update one input register from the local application. */
ld_modbus_status_t ld_modbus_server_map_set_input_register(ld_modbus_server_map_t *map,
                                                           uint16_t address,
                                                           uint16_t value)
{
    uint16_t index;
    ld_modbus_status_t status;

    if(map == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(map->input_registers == NULL)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    status = ld_modbus_server_map_index(address, map->input_registers_start,
                                        map->input_registers_count, &index);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    map->input_registers[index] = value;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Process functions 01 and 02. */
static ld_modbus_status_t ld_modbus_server_read_bits(const ld_modbus_server_map_t *map,
                                                     const uint8_t *request,
                                                     uint8_t *response,
                                                     size_t response_capacity,
                                                     size_t *response_length)
{
    const uint8_t *source;
    uint16_t start;
    uint16_t count;
    uint16_t address = ld_modbus_get_u16(&request[1]);
    uint16_t quantity = ld_modbus_get_u16(&request[3]);
    size_t byte_count;
    uint16_t index;

    if(request[0] == LD_MODBUS_FC_READ_COILS)
    {
        source = map->coils;
        start = map->coils_start;
        count = map->coils_count;
    }
    else
    {
        source = map->discrete_inputs;
        start = map->discrete_inputs_start;
        count = map->discrete_inputs_count;
    }
    if(quantity == 0U || quantity > 2000U)
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE,
                                          response, response_capacity, response_length);
    if(source == NULL || !ld_modbus_server_range_is_valid(address, quantity, start, count))
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
                                          response, response_capacity, response_length);

    byte_count = ((size_t)quantity + 7U) / 8U;
    if(response_capacity < byte_count + 2U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;
    response[0] = request[0];
    response[1] = (uint8_t)byte_count;
    memset(&response[2], 0, byte_count);
    for(index = 0U; index < quantity; ++index)
    {
        if(source[(uint32_t)address - start + index] != 0U)
            response[2U + index / 8U] |= (uint8_t)(1U << (index & 7U));
    }
    *response_length = byte_count + 2U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Process functions 03 and 04. */
static ld_modbus_status_t ld_modbus_server_read_registers(
    const ld_modbus_server_map_t *map,
    const uint8_t *request,
    uint8_t *response,
    size_t response_capacity,
    size_t *response_length)
{
    const uint16_t *source;
    uint16_t start;
    uint16_t count;
    uint16_t address = ld_modbus_get_u16(&request[1]);
    uint16_t quantity = ld_modbus_get_u16(&request[3]);
    uint16_t index;

    if(request[0] == LD_MODBUS_FC_READ_HOLDING_REGISTERS)
    {
        source = map->holding_registers;
        start = map->holding_registers_start;
        count = map->holding_registers_count;
    }
    else
    {
        source = map->input_registers;
        start = map->input_registers_start;
        count = map->input_registers_count;
    }
    if(quantity == 0U || quantity > 125U)
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE,
                                          response, response_capacity, response_length);
    if(source == NULL || !ld_modbus_server_range_is_valid(address, quantity, start, count))
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
                                          response, response_capacity, response_length);
    if(response_capacity < 2U + (size_t)quantity * 2U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;

    response[0] = request[0];
    response[1] = (uint8_t)(quantity * 2U);
    for(index = 0U; index < quantity; ++index)
        ld_modbus_put_u16(&response[2U + (size_t)index * 2U],
                          source[(uint32_t)address - start + index]);
    *response_length = 2U + (size_t)quantity * 2U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Process function 05. */
static ld_modbus_status_t ld_modbus_server_write_single_coil(
    const ld_modbus_server_map_t *map,
    const uint8_t *request,
    uint8_t *response,
    size_t response_capacity,
    size_t *response_length)
{
    uint16_t address = ld_modbus_get_u16(&request[1]);
    uint16_t encoded_value = ld_modbus_get_u16(&request[3]);

    if(encoded_value != 0x0000U && encoded_value != 0xFF00U)
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE,
                                          response, response_capacity, response_length);
    if(map->coils == NULL ||
       !ld_modbus_server_range_is_valid(address, 1U, map->coils_start, map->coils_count))
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
                                          response, response_capacity, response_length);
    if(response_capacity < 5U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;

    map->coils[(uint32_t)address - map->coils_start] = encoded_value == 0xFF00U ? 1U : 0U;
    memcpy(response, request, 5U);
    *response_length = 5U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Process function 06. */
static ld_modbus_status_t ld_modbus_server_write_single_register(
    const ld_modbus_server_map_t *map,
    const uint8_t *request,
    uint8_t *response,
    size_t response_capacity,
    size_t *response_length)
{
    uint16_t address = ld_modbus_get_u16(&request[1]);

    if(map->holding_registers == NULL ||
       !ld_modbus_server_range_is_valid(address, 1U,
                                        map->holding_registers_start,
                                        map->holding_registers_count))
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
                                          response, response_capacity, response_length);
    if(response_capacity < 5U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;
    map->holding_registers[(uint32_t)address - map->holding_registers_start] =
        ld_modbus_get_u16(&request[3]);
    memcpy(response, request, 5U);
    *response_length = 5U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Process function 0F after validating the entire request. */
static ld_modbus_status_t ld_modbus_server_write_multiple_coils(
    const ld_modbus_server_map_t *map,
    const uint8_t *request,
    size_t request_length,
    uint8_t *response,
    size_t response_capacity,
    size_t *response_length)
{
    uint16_t address = ld_modbus_get_u16(&request[1]);
    uint16_t quantity = ld_modbus_get_u16(&request[3]);
    size_t expected_bytes = ((size_t)quantity + 7U) / 8U;
    uint16_t index;

    if(quantity == 0U || quantity > 1968U || request[5] != expected_bytes ||
       request_length != expected_bytes + 6U)
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE,
                                          response, response_capacity, response_length);
    if(map->coils == NULL || !ld_modbus_server_range_is_valid(address, quantity,
                                                               map->coils_start,
                                                               map->coils_count))
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
                                          response, response_capacity, response_length);
    if(response_capacity < 5U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;
    for(index = 0U; index < quantity; ++index)
        map->coils[(uint32_t)address - map->coils_start + index] =
            (uint8_t)((request[6U + index / 8U] >> (index & 7U)) & 1U);
    response[0] = request[0];
    memcpy(&response[1], &request[1], 4U);
    *response_length = 5U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Process function 10 after validating the entire request. */
static ld_modbus_status_t ld_modbus_server_write_multiple_registers(
    const ld_modbus_server_map_t *map,
    const uint8_t *request,
    size_t request_length,
    uint8_t *response,
    size_t response_capacity,
    size_t *response_length)
{
    uint16_t address = ld_modbus_get_u16(&request[1]);
    uint16_t quantity = ld_modbus_get_u16(&request[3]);
    size_t expected_bytes = (size_t)quantity * 2U;
    uint16_t index;

    if(quantity == 0U || quantity > 123U || request[5] != expected_bytes ||
       request_length != expected_bytes + 6U)
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE,
                                          response, response_capacity, response_length);
    if(map->holding_registers == NULL ||
       !ld_modbus_server_range_is_valid(address, quantity,
                                        map->holding_registers_start,
                                        map->holding_registers_count))
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
                                          response, response_capacity, response_length);
    if(response_capacity < 5U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;
    for(index = 0U; index < quantity; ++index)
        map->holding_registers[(uint32_t)address - map->holding_registers_start + index] =
            ld_modbus_get_u16(&request[6U + (size_t)index * 2U]);
    response[0] = request[0];
    memcpy(&response[1], &request[1], 4U);
    *response_length = 5U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Process function 16 mask write register. */
static ld_modbus_status_t ld_modbus_server_mask_write_register(
    const ld_modbus_server_map_t *map,
    const uint8_t *request,
    uint8_t *response,
    size_t response_capacity,
    size_t *response_length)
{
    uint16_t address = ld_modbus_get_u16(&request[1]);
    uint16_t and_mask = ld_modbus_get_u16(&request[3]);
    uint16_t or_mask = ld_modbus_get_u16(&request[5]);
    uint16_t *value;

    if(map->holding_registers == NULL ||
       !ld_modbus_server_range_is_valid(address, 1U,
                                        map->holding_registers_start,
                                        map->holding_registers_count))
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
                                          response, response_capacity, response_length);
    if(response_capacity < 7U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;
    value = &map->holding_registers[(uint32_t)address - map->holding_registers_start];
    *value = (uint16_t)((*value & and_mask) | (or_mask & (uint16_t)~and_mask));
    memcpy(response, request, 7U);
    *response_length = 7U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Process function 17 with write-before-read semantics. */
static ld_modbus_status_t ld_modbus_server_write_read_registers(
    const ld_modbus_server_map_t *map,
    const uint8_t *request,
    size_t request_length,
    uint8_t *response,
    size_t response_capacity,
    size_t *response_length)
{
    uint16_t read_address = ld_modbus_get_u16(&request[1]);
    uint16_t read_quantity = ld_modbus_get_u16(&request[3]);
    uint16_t write_address = ld_modbus_get_u16(&request[5]);
    uint16_t write_quantity = ld_modbus_get_u16(&request[7]);
    size_t write_bytes = (size_t)write_quantity * 2U;
    uint16_t index;

    if(read_quantity == 0U || read_quantity > 125U || write_quantity == 0U ||
       write_quantity > 121U || request[9] != write_bytes ||
       request_length != write_bytes + 10U)
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE,
                                          response, response_capacity, response_length);
    if(map->holding_registers == NULL ||
       !ld_modbus_server_range_is_valid(read_address, read_quantity,
                                        map->holding_registers_start,
                                        map->holding_registers_count) ||
       !ld_modbus_server_range_is_valid(write_address, write_quantity,
                                        map->holding_registers_start,
                                        map->holding_registers_count))
        return ld_modbus_server_exception(request[0], LD_MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
                                          response, response_capacity, response_length);
    if(response_capacity < 2U + (size_t)read_quantity * 2U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;

    for(index = 0U; index < write_quantity; ++index)
        map->holding_registers[(uint32_t)write_address - map->holding_registers_start + index] =
            ld_modbus_get_u16(&request[10U + (size_t)index * 2U]);
    response[0] = request[0];
    response[1] = (uint8_t)(read_quantity * 2U);
    for(index = 0U; index < read_quantity; ++index)
        ld_modbus_put_u16(&response[2U + (size_t)index * 2U],
                          map->holding_registers[(uint32_t)read_address -
                                                 map->holding_registers_start + index]);
    *response_length = 2U + (size_t)read_quantity * 2U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Process one request PDU against a caller-owned server map. */
ld_modbus_status_t ld_modbus_server_process_pdu(const ld_modbus_server_map_t *map,
                                                const uint8_t *request_pdu,
                                                size_t request_length,
                                                uint8_t *response_pdu,
                                                size_t response_capacity,
                                                size_t *response_length)
{
    uint8_t function;

    if(map == NULL || request_pdu == NULL || response_pdu == NULL ||
       response_length == NULL || request_length == 0U ||
       request_length > LD_MODBUS_MAX_PDU_LENGTH)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    *response_length = 0U;
    function = request_pdu[0];

    switch(function)
    {
    case LD_MODBUS_FC_READ_COILS:
    case LD_MODBUS_FC_READ_DISCRETE_INPUTS:
        if(request_length != 5U)
            return LD_MODBUS_STATUS_MALFORMED_FRAME;
        return ld_modbus_server_read_bits(map, request_pdu, response_pdu,
                                          response_capacity, response_length);
    case LD_MODBUS_FC_READ_HOLDING_REGISTERS:
    case LD_MODBUS_FC_READ_INPUT_REGISTERS:
        if(request_length != 5U)
            return LD_MODBUS_STATUS_MALFORMED_FRAME;
        return ld_modbus_server_read_registers(map, request_pdu, response_pdu,
                                               response_capacity, response_length);
    case LD_MODBUS_FC_WRITE_SINGLE_COIL:
        if(request_length != 5U)
            return LD_MODBUS_STATUS_MALFORMED_FRAME;
        return ld_modbus_server_write_single_coil(map, request_pdu, response_pdu,
                                                  response_capacity, response_length);
    case LD_MODBUS_FC_WRITE_SINGLE_REGISTER:
        if(request_length != 5U)
            return LD_MODBUS_STATUS_MALFORMED_FRAME;
        return ld_modbus_server_write_single_register(map, request_pdu, response_pdu,
                                                      response_capacity, response_length);
    case LD_MODBUS_FC_WRITE_MULTIPLE_COILS:
        if(request_length < 7U)
            return LD_MODBUS_STATUS_MALFORMED_FRAME;
        return ld_modbus_server_write_multiple_coils(map, request_pdu, request_length,
                                                     response_pdu, response_capacity,
                                                     response_length);
    case LD_MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
        if(request_length < 8U)
            return LD_MODBUS_STATUS_MALFORMED_FRAME;
        return ld_modbus_server_write_multiple_registers(map, request_pdu, request_length,
                                                         response_pdu, response_capacity,
                                                         response_length);
    case LD_MODBUS_FC_MASK_WRITE_REGISTER:
        if(request_length != 7U)
            return LD_MODBUS_STATUS_MALFORMED_FRAME;
        return ld_modbus_server_mask_write_register(map, request_pdu, response_pdu,
                                                    response_capacity, response_length);
    case LD_MODBUS_FC_WRITE_READ_MULTIPLE_REGISTERS:
        if(request_length < 12U)
            return LD_MODBUS_STATUS_MALFORMED_FRAME;
        return ld_modbus_server_write_read_registers(map, request_pdu, request_length,
                                                     response_pdu, response_capacity,
                                                     response_length);
    default:
        return ld_modbus_server_exception(function, LD_MODBUS_EXCEPTION_ILLEGAL_FUNCTION,
                                          response_pdu, response_capacity, response_length);
    }
}

/** @brief Process one complete RTU ADU with unit filtering and broadcast rules. */
ld_modbus_status_t ld_modbus_server_process_rtu_adu(
    const ld_modbus_server_map_t *map,
    uint8_t local_unit_id,
    const uint8_t *request_adu,
    size_t request_length,
    uint8_t *response_adu,
    size_t response_capacity,
    size_t *response_length,
    ld_modbus_server_action_t *action)
{
    ld_modbus_adu_view_t request_view;
    ld_modbus_status_t status;
    size_t response_pdu_length;

    if(map == NULL || request_adu == NULL || response_adu == NULL ||
       response_length == NULL || action == NULL ||
       local_unit_id == LD_MODBUS_BROADCAST_UNIT_ID ||
       local_unit_id > LD_MODBUS_MAX_UNIT_ID)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    *response_length = 0U;
    *action = LD_MODBUS_SERVER_ACTION_IGNORED;

    status = ld_modbus_rtu_decode(request_adu, request_length, &request_view);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    if(request_view.unit_id != local_unit_id &&
       request_view.unit_id != LD_MODBUS_BROADCAST_UNIT_ID)
        return LD_MODBUS_STATUS_OK;
    if(request_view.unit_id == LD_MODBUS_BROADCAST_UNIT_ID &&
       !ld_modbus_server_is_broadcast_write(request_view.pdu[0]))
        return LD_MODBUS_STATUS_OK;

    status = ld_modbus_server_process_pdu(map,
                                          request_view.pdu,
                                          request_view.pdu_length,
                                          response_adu,
                                          response_capacity,
                                          &response_pdu_length);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    if(request_view.unit_id == LD_MODBUS_BROADCAST_UNIT_ID)
    {
        *action = LD_MODBUS_SERVER_ACTION_BROADCAST_APPLIED;
        return LD_MODBUS_STATUS_OK;
    }

    status = ld_modbus_rtu_encode(local_unit_id,
                                  response_adu,
                                  response_pdu_length,
                                  response_adu,
                                  response_capacity,
                                  response_length);
    if(status == LD_MODBUS_STATUS_OK)
        *action = LD_MODBUS_SERVER_ACTION_REPLY;
    return status;
}

/** @brief Process one complete TCP ADU while preserving its MBAP correlation fields. */
ld_modbus_status_t ld_modbus_server_process_tcp_adu(
    const ld_modbus_server_map_t *map,
    const uint8_t *request_adu,
    size_t request_length,
    uint8_t *response_adu,
    size_t response_capacity,
    size_t *response_length)
{
    ld_modbus_adu_view_t request_view;
    ld_modbus_status_t status;
    size_t response_pdu_length;

    if(map == NULL || request_adu == NULL || response_adu == NULL ||
       response_length == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    *response_length = 0U;
    status = ld_modbus_tcp_decode(request_adu, request_length, &request_view);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    status = ld_modbus_server_process_pdu(map,
                                          request_view.pdu,
                                          request_view.pdu_length,
                                          response_adu,
                                          response_capacity,
                                          &response_pdu_length);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    return ld_modbus_tcp_encode(request_view.transaction_id,
                                request_view.unit_id,
                                response_adu,
                                response_pdu_length,
                                response_adu,
                                response_capacity,
                                response_length);
}
