/**
 * @file ld_modbus_core.c
 * @brief Transport-independent Modbus RTU and TCP ADU codecs.
 */

#include "ld_modbus.h"

#include <string.h>

/** @brief Validate a unit identifier accepted by RTU and TCP gateways. */
static int ld_modbus_unit_id_is_valid(uint8_t unit_id)
{
    return unit_id <= LD_MODBUS_MAX_UNIT_ID;
}

/** @brief Calculate the Modbus RTU CRC-16 for a bounded byte sequence. */
uint16_t ld_modbus_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;
    unsigned int bit;

    if(data == NULL && length != 0U)
        return 0U;

    for(index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for(bit = 0U; bit < 8U; ++bit)
        {
            if((crc & 1U) != 0U)
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            else
                crc >>= 1U;
        }
    }
    return crc;
}

/** @brief Encode a PDU as a complete Modbus RTU ADU. */
ld_modbus_status_t ld_modbus_rtu_encode(uint8_t unit_id,
                                        const uint8_t *pdu,
                                        size_t pdu_length,
                                        uint8_t *output,
                                        size_t output_capacity,
                                        size_t *output_length)
{
    size_t required;
    uint16_t crc;

    if(pdu == NULL || output == NULL || output_length == NULL ||
       pdu_length == 0U || pdu_length > LD_MODBUS_MAX_PDU_LENGTH ||
       !ld_modbus_unit_id_is_valid(unit_id))
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;

    required = pdu_length + 3U;
    if(output_capacity < required)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;

    /* Move before writing the header: pdu may alias output exactly. */
    memmove(&output[1], pdu, pdu_length);
    output[0] = unit_id;
    crc = ld_modbus_crc16(output, pdu_length + 1U);
    output[pdu_length + 1U] = (uint8_t)(crc & 0xFFU);
    output[pdu_length + 2U] = (uint8_t)(crc >> 8U);
    *output_length = required;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Validate and expose the fields of a complete Modbus RTU ADU. */
ld_modbus_status_t ld_modbus_rtu_decode(const uint8_t *adu,
                                        size_t adu_length,
                                        ld_modbus_adu_view_t *view)
{
    uint16_t expected_crc;
    uint16_t received_crc;

    if(adu == NULL || view == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(adu_length < 4U || adu_length > LD_MODBUS_RTU_MAX_ADU_LENGTH)
        return LD_MODBUS_STATUS_MALFORMED_FRAME;
    if(!ld_modbus_unit_id_is_valid(adu[0]))
        return LD_MODBUS_STATUS_MALFORMED_FRAME;

    expected_crc = ld_modbus_crc16(adu, adu_length - 2U);
    received_crc = (uint16_t)adu[adu_length - 2U] |
                   (uint16_t)((uint16_t)adu[adu_length - 1U] << 8U);
    if(expected_crc != received_crc)
        return LD_MODBUS_STATUS_BAD_CRC;

    view->transaction_id = 0U;
    view->unit_id = adu[0];
    view->pdu = &adu[1];
    view->pdu_length = adu_length - 3U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Encode a PDU as a complete Modbus TCP ADU. */
ld_modbus_status_t ld_modbus_tcp_encode(uint16_t transaction_id,
                                        uint8_t unit_id,
                                        const uint8_t *pdu,
                                        size_t pdu_length,
                                        uint8_t *output,
                                        size_t output_capacity,
                                        size_t *output_length)
{
    size_t required;
    uint16_t mbap_length;

    if(pdu == NULL || output == NULL || output_length == NULL ||
       pdu_length == 0U || pdu_length > LD_MODBUS_MAX_PDU_LENGTH ||
       !ld_modbus_unit_id_is_valid(unit_id))
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;

    required = pdu_length + 7U;
    if(output_capacity < required)
        return LD_MODBUS_STATUS_BUFFER_TOO_SMALL;

    mbap_length = (uint16_t)(pdu_length + 1U);
    /* Move before writing MBAP: pdu may alias output exactly. */
    memmove(&output[7], pdu, pdu_length);
    output[0] = (uint8_t)(transaction_id >> 8U);
    output[1] = (uint8_t)transaction_id;
    output[2] = 0U;
    output[3] = 0U;
    output[4] = (uint8_t)(mbap_length >> 8U);
    output[5] = (uint8_t)mbap_length;
    output[6] = unit_id;
    *output_length = required;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Validate and expose the fields of a complete Modbus TCP ADU. */
ld_modbus_status_t ld_modbus_tcp_decode(const uint8_t *adu,
                                        size_t adu_length,
                                        ld_modbus_adu_view_t *view)
{
    uint16_t protocol_id;
    uint16_t mbap_length;

    if(adu == NULL || view == NULL)
        return LD_MODBUS_STATUS_INVALID_ARGUMENT;
    if(adu_length < 8U || adu_length > LD_MODBUS_TCP_MAX_ADU_LENGTH)
        return LD_MODBUS_STATUS_MALFORMED_FRAME;

    protocol_id = (uint16_t)((uint16_t)adu[2] << 8U) | adu[3];
    if(protocol_id != 0U)
        return LD_MODBUS_STATUS_BAD_PROTOCOL_ID;

    mbap_length = (uint16_t)((uint16_t)adu[4] << 8U) | adu[5];
    if(mbap_length < 2U || (size_t)mbap_length + 6U != adu_length ||
       !ld_modbus_unit_id_is_valid(adu[6]))
        return LD_MODBUS_STATUS_MALFORMED_FRAME;

    view->transaction_id = (uint16_t)((uint16_t)adu[0] << 8U) | adu[1];
    view->unit_id = adu[6];
    view->pdu = &adu[7];
    view->pdu_length = adu_length - 7U;
    return LD_MODBUS_STATUS_OK;
}

/** @brief Convert a status code to a stable diagnostic string. */
const char *ld_modbus_status_string(ld_modbus_status_t status)
{
    switch(status)
    {
    case LD_MODBUS_STATUS_OK: return "ok";
    case LD_MODBUS_STATUS_INVALID_ARGUMENT: return "invalid argument";
    case LD_MODBUS_STATUS_BUFFER_TOO_SMALL: return "buffer too small";
    case LD_MODBUS_STATUS_MALFORMED_FRAME: return "malformed frame";
    case LD_MODBUS_STATUS_BAD_CRC: return "bad CRC";
    case LD_MODBUS_STATUS_BAD_PROTOCOL_ID: return "bad protocol identifier";
    case LD_MODBUS_STATUS_UNIT_MISMATCH: return "unit mismatch";
    case LD_MODBUS_STATUS_TRANSACTION_MISMATCH: return "transaction mismatch";
    case LD_MODBUS_STATUS_FUNCTION_MISMATCH: return "function mismatch";
    case LD_MODBUS_STATUS_EXCEPTION_RESPONSE: return "exception response";
    case LD_MODBUS_STATUS_RANGE_ERROR: return "range error";
    case LD_MODBUS_STATUS_NOT_SUPPORTED: return "not supported";
    default: return "unknown status";
    }
}
