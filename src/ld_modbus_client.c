/**
 * @file ld_modbus_client.c
 * @brief Stateless, bounded Modbus client PDU helpers.
 */

#include "ld_modbus_client.h"

/** @brief Store a 16-bit integer in Modbus big-endian byte order. */
static void ld_modbus_put_u16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value >> 8U);
    destination[1] = (uint8_t)value;
}

/** @brief Read a 16-bit integer from Modbus big-endian byte order. */
static uint16_t ld_modbus_get_u16(const uint8_t *source)
{
    return (uint16_t)((uint16_t)source[0] << 8U) | source[1];
}

/** @brief Validate and expose a Modbus exception response. */
static ld_modbus_status_t ld_modbus_client_check_function(uint8_t expected_function,
                                                          const uint8_t *pdu,
                                                          size_t pdu_length,
                                                          uint8_t *exception_code)
{
    if(pdu == NULL || pdu_length == 0U)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(exception_code != NULL)
        *exception_code = 0U;
    if(pdu[0] == (uint8_t)(expected_function | 0x80U))
    {
        if(pdu_length != 2U)
            return LD_MODBUS_STATUS_MALFORMED_FRAME;
        if(exception_code != NULL)
            *exception_code = pdu[1];
        return LD_MODBUS_STATUS_EXCEPTION_RESPONSE;
    }
    if(pdu[0] != expected_function)
        return LD_MODBUS_STATUS_FUNCTION_MISMATCH;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Build an FC01, FC02, FC03, or FC04 read request PDU. */
ld_modbus_status_t ld_modbus_client_build_read_request(uint8_t function,
                                                       uint16_t address,
                                                       uint16_t quantity,
                                                       uint8_t *pdu,
                                                       size_t pdu_capacity,
                                                       size_t *pdu_length)
{
    uint16_t maximum;

    if(pdu == NULL || pdu_length == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(function == LD_MODBUS_FC_READ_COILS ||
       function == LD_MODBUS_FC_READ_DISCRETE_INPUTS)
        maximum = 2000U;
    else if(function == LD_MODBUS_FC_READ_HOLDING_REGISTERS ||
            function == LD_MODBUS_FC_READ_INPUT_REGISTERS)
        maximum = 125U;
    else
        return LD_MODBUS_STATUS_NOT_SUPPORTED;
    if(quantity == 0U || quantity > maximum)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    if(pdu_capacity < 5U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;

    pdu[0] = function;
    ld_modbus_put_u16(&pdu[1], address);
    ld_modbus_put_u16(&pdu[3], quantity);
    *pdu_length = 5U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Build an FC05 write-single-coil request PDU. */
ld_modbus_status_t ld_modbus_client_build_write_single_coil(uint16_t address,
                                                            uint8_t value,
                                                            uint8_t *pdu,
                                                            size_t pdu_capacity,
                                                            size_t *pdu_length)
{
    if(pdu == NULL || pdu_length == NULL || value > 1U)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(pdu_capacity < 5U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;
    pdu[0] = LD_MODBUS_FC_WRITE_SINGLE_COIL;
    ld_modbus_put_u16(&pdu[1], address);
    ld_modbus_put_u16(&pdu[3], value != 0U ? 0xFF00U : 0x0000U);
    *pdu_length = 5U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Build an FC06 write-single-register request PDU. */
ld_modbus_status_t ld_modbus_client_build_write_single_register(uint16_t address,
                                                                uint16_t value,
                                                                uint8_t *pdu,
                                                                size_t pdu_capacity,
                                                                size_t *pdu_length)
{
    if(pdu == NULL || pdu_length == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(pdu_capacity < 5U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;
    pdu[0] = LD_MODBUS_FC_WRITE_SINGLE_REGISTER;
    ld_modbus_put_u16(&pdu[1], address);
    ld_modbus_put_u16(&pdu[3], value);
    *pdu_length = 5U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Build an FC0F write-multiple-coils request PDU. */
ld_modbus_status_t ld_modbus_client_build_write_multiple_coils(uint16_t address,
                                                               const uint8_t *values,
                                                               uint16_t quantity,
                                                               uint8_t *pdu,
                                                               size_t pdu_capacity,
                                                               size_t *pdu_length)
{
    size_t byte_count;
    size_t required;
    uint16_t index;

    if(values == NULL || pdu == NULL || pdu_length == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(quantity == 0U || quantity > 1968U)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    byte_count = ((size_t)quantity + 7U) / 8U;
    required = 6U + byte_count;
    if(pdu_capacity < required)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;

    pdu[0] = LD_MODBUS_FC_WRITE_MULTIPLE_COILS;
    ld_modbus_put_u16(&pdu[1], address);
    ld_modbus_put_u16(&pdu[3], quantity);
    pdu[5] = (uint8_t)byte_count;
    for(index = 0U; index < quantity; ++index)
    {
        if(values[index] > 1U)
            return LD_MODBUS_STATUS_INVALID_ARGUMENT;
        if((index & 7U) == 0U)
            pdu[6U + index / 8U] = 0U;
        if(values[index] != 0U)
            pdu[6U + index / 8U] |= (uint8_t)(1U << (index & 7U));
    }
    *pdu_length = required;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Build an FC10 write-multiple-registers request PDU. */
ld_modbus_status_t ld_modbus_client_build_write_multiple_registers(
    uint16_t address,
    const uint16_t *values,
    uint16_t quantity,
    uint8_t *pdu,
    size_t pdu_capacity,
    size_t *pdu_length)
{
    size_t required;
    uint16_t index;

    if(values == NULL || pdu == NULL || pdu_length == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(quantity == 0U || quantity > 123U)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    required = 6U + (size_t)quantity * 2U;
    if(pdu_capacity < required)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;

    pdu[0] = LD_MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
    ld_modbus_put_u16(&pdu[1], address);
    ld_modbus_put_u16(&pdu[3], quantity);
    pdu[5] = (uint8_t)(quantity * 2U);
    for(index = 0U; index < quantity; ++index)
        ld_modbus_put_u16(&pdu[6U + (size_t)index * 2U], values[index]);
    *pdu_length = required;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Build function 16 from one address and two masks. */
/** @brief Build an FC16 mask-write-register request PDU. */
ld_modbus_status_t ld_modbus_client_build_mask_write_register(
    uint16_t address,
    uint16_t and_mask,
    uint16_t or_mask,
    uint8_t *pdu,
    size_t pdu_capacity,
    size_t *pdu_length)
{
    if(pdu == NULL || pdu_length == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(pdu_capacity < 7U)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;
    pdu[0] = LD_MODBUS_FC_MASK_WRITE_REGISTER;
    ld_modbus_put_u16(&pdu[1], address);
    ld_modbus_put_u16(&pdu[3], and_mask);
    ld_modbus_put_u16(&pdu[5], or_mask);
    *pdu_length = 7U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Build function 17 with bounded read and write register counts. */
/** @brief Build an FC17 combined write/read-registers request PDU. */
ld_modbus_status_t ld_modbus_client_build_write_read_multiple_registers(
    uint16_t read_address,
    uint16_t read_quantity,
    uint16_t write_address,
    const uint16_t *write_values,
    uint16_t write_quantity,
    uint8_t *pdu,
    size_t pdu_capacity,
    size_t *pdu_length)
{
    size_t required;
    uint16_t index;

    if(write_values == NULL || pdu == NULL || pdu_length == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(read_quantity == 0U || read_quantity > 125U ||
       write_quantity == 0U || write_quantity > 121U)
        return LD_MODBUS_STATUS_RANGE_ERROR;
    required = 10U + (size_t)write_quantity * 2U;
    if(pdu_capacity < required)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;

    pdu[0] = LD_MODBUS_FC_WRITE_READ_MULTIPLE_REGISTERS;
    ld_modbus_put_u16(&pdu[1], read_address);
    ld_modbus_put_u16(&pdu[3], read_quantity);
    ld_modbus_put_u16(&pdu[5], write_address);
    ld_modbus_put_u16(&pdu[7], write_quantity);
    pdu[9] = (uint8_t)(write_quantity * 2U);
    for(index = 0U; index < write_quantity; ++index)
        ld_modbus_put_u16(&pdu[10U + (size_t)index * 2U], write_values[index]);
    *pdu_length = required;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Parse an FC01 or FC02 bit-read response PDU. */
ld_modbus_status_t ld_modbus_client_parse_read_bits_response(
    uint8_t expected_function,
    uint16_t expected_quantity,
    const uint8_t *pdu,
    size_t pdu_length,
    uint8_t *values,
    size_t values_capacity,
    uint8_t *exception_code)
{
    ld_modbus_status_t status;
    size_t expected_bytes;
    uint16_t index;

    if(values == NULL || expected_quantity == 0U || values_capacity < expected_quantity)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    status = ld_modbus_client_check_function(expected_function, pdu, pdu_length,
                                             exception_code);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    if(expected_function != LD_MODBUS_FC_READ_COILS &&
       expected_function != LD_MODBUS_FC_READ_DISCRETE_INPUTS)
        return LD_MODBUS_STATUS_NOT_SUPPORTED;
    expected_bytes = ((size_t)expected_quantity + 7U) / 8U;
    if(pdu_length != expected_bytes + 2U || pdu[1] != expected_bytes)
        return LD_MODBUS_STATUS_MALFORMED_FRAME;
    for(index = 0U; index < expected_quantity; ++index)
        values[index] = (uint8_t)((pdu[2U + index / 8U] >> (index & 7U)) & 1U);
    return LD_MODBUS_STATUS_OK;
}

/** @brief Parse an FC03, FC04, or FC17 register-read response PDU. */
ld_modbus_status_t ld_modbus_client_parse_read_registers_response(
    uint8_t expected_function,
    uint16_t expected_quantity,
    const uint8_t *pdu,
    size_t pdu_length,
    uint16_t *values,
    size_t values_capacity,
    uint8_t *exception_code)
{
    ld_modbus_status_t status;
    size_t expected_bytes;
    uint16_t index;

    if(values == NULL || expected_quantity == 0U || values_capacity < expected_quantity)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    status = ld_modbus_client_check_function(expected_function, pdu, pdu_length,
                                             exception_code);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    if(expected_function != LD_MODBUS_FC_READ_HOLDING_REGISTERS &&
       expected_function != LD_MODBUS_FC_READ_INPUT_REGISTERS &&
       expected_function != LD_MODBUS_FC_WRITE_READ_MULTIPLE_REGISTERS)
        return LD_MODBUS_STATUS_NOT_SUPPORTED;
    expected_bytes = (size_t)expected_quantity * 2U;
    if(pdu_length != expected_bytes + 2U || pdu[1] != expected_bytes)
        return LD_MODBUS_STATUS_MALFORMED_FRAME;
    for(index = 0U; index < expected_quantity; ++index)
        values[index] = ld_modbus_get_u16(&pdu[2U + (size_t)index * 2U]);
    return LD_MODBUS_STATUS_OK;
}

/** @brief Validate the echoed response for FC05, FC06, FC0F, or FC10. */
ld_modbus_status_t ld_modbus_client_parse_write_response(uint8_t expected_function,
                                                         uint16_t expected_address,
                                                         uint16_t expected_value_or_quantity,
                                                         const uint8_t *pdu,
                                                         size_t pdu_length,
                                                         uint8_t *exception_code)
{
    ld_modbus_status_t status;

    status = ld_modbus_client_check_function(expected_function, pdu, pdu_length,
                                             exception_code);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    if(expected_function != LD_MODBUS_FC_WRITE_SINGLE_COIL &&
       expected_function != LD_MODBUS_FC_WRITE_SINGLE_REGISTER &&
       expected_function != LD_MODBUS_FC_WRITE_MULTIPLE_COILS &&
       expected_function != LD_MODBUS_FC_WRITE_MULTIPLE_REGISTERS)
        return LD_MODBUS_STATUS_NOT_SUPPORTED;
    if(pdu_length != 5U)
        return LD_MODBUS_STATUS_MALFORMED_FRAME;
    if(ld_modbus_get_u16(&pdu[1]) != expected_address ||
       ld_modbus_get_u16(&pdu[3]) != expected_value_or_quantity)
        return LD_MODBUS_STATUS_MALFORMED_FRAME;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Validate a function 16 response against its complete request echo. */
/** @brief Validate the echoed response for FC16 mask write. */
ld_modbus_status_t ld_modbus_client_parse_mask_write_response(
    uint16_t expected_address,
    uint16_t expected_and_mask,
    uint16_t expected_or_mask,
    const uint8_t *pdu,
    size_t pdu_length,
    uint8_t *exception_code)
{
    ld_modbus_status_t status;

    status = ld_modbus_client_check_function(LD_MODBUS_FC_MASK_WRITE_REGISTER,
                                             pdu, pdu_length, exception_code);
    if(status != LD_MODBUS_STATUS_OK)
        return status;
    if(pdu_length != 7U)
        return LD_MODBUS_STATUS_MALFORMED_FRAME;
    if(ld_modbus_get_u16(&pdu[1]) != expected_address ||
       ld_modbus_get_u16(&pdu[3]) != expected_and_mask ||
       ld_modbus_get_u16(&pdu[5]) != expected_or_mask)
        return LD_MODBUS_STATUS_MALFORMED_FRAME;
    return LD_MODBUS_STATUS_OK;
}
