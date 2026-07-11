/**
 * @file ld_modbus.h
 * @brief Common types and RTU/TCP ADU codecs for ld_modbus.
 */

#ifndef LD_MODBUS_H
#define LD_MODBUS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LD_MODBUS_VERSION_MAJOR 0U
#define LD_MODBUS_VERSION_MINOR 1U
#define LD_MODBUS_VERSION_PATCH 0U

#define LD_MODBUS_MAX_PDU_LENGTH 253U
#define LD_MODBUS_RTU_MAX_ADU_LENGTH 256U
#define LD_MODBUS_TCP_MAX_ADU_LENGTH 260U
#define LD_MODBUS_BROADCAST_UNIT_ID 0U
#define LD_MODBUS_MAX_UNIT_ID 247U

/** @brief Library status returned by every fallible operation. */
typedef enum
{
    LD_MODBUS_STATUS_OK = 0,
    LD_MODBUS_STATUS_INVALID_ARGUMENT,
    LD_MODBUS_STATUS_BUFFER_TOO_SMALL,
    LD_MODBUS_STATUS_MALFORMED_FRAME,
    LD_MODBUS_STATUS_BAD_CRC,
    LD_MODBUS_STATUS_BAD_PROTOCOL_ID,
    LD_MODBUS_STATUS_UNIT_MISMATCH,
    LD_MODBUS_STATUS_TRANSACTION_MISMATCH,
    LD_MODBUS_STATUS_FUNCTION_MISMATCH,
    LD_MODBUS_STATUS_EXCEPTION_RESPONSE,
    LD_MODBUS_STATUS_RANGE_ERROR,
    LD_MODBUS_STATUS_NOT_SUPPORTED
} ld_modbus_status_t;

/** @brief Public function-code constants supported by v0.1. */
typedef enum
{
    LD_MODBUS_FC_READ_COILS = 0x01,
    LD_MODBUS_FC_READ_DISCRETE_INPUTS = 0x02,
    LD_MODBUS_FC_READ_HOLDING_REGISTERS = 0x03,
    LD_MODBUS_FC_READ_INPUT_REGISTERS = 0x04,
    LD_MODBUS_FC_WRITE_SINGLE_COIL = 0x05,
    LD_MODBUS_FC_WRITE_SINGLE_REGISTER = 0x06,
    LD_MODBUS_FC_WRITE_MULTIPLE_COILS = 0x0F,
    LD_MODBUS_FC_WRITE_MULTIPLE_REGISTERS = 0x10,
    LD_MODBUS_FC_MASK_WRITE_REGISTER = 0x16,
    LD_MODBUS_FC_WRITE_READ_MULTIPLE_REGISTERS = 0x17
} ld_modbus_function_t;

/** @brief Standard Modbus exception codes used in server responses. */
typedef enum
{
    LD_MODBUS_EXCEPTION_ILLEGAL_FUNCTION = 0x01,
    LD_MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS = 0x02,
    LD_MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE = 0x03,
    LD_MODBUS_EXCEPTION_SERVER_DEVICE_FAILURE = 0x04,
    LD_MODBUS_EXCEPTION_ACKNOWLEDGE = 0x05,
    LD_MODBUS_EXCEPTION_SERVER_DEVICE_BUSY = 0x06,
    LD_MODBUS_EXCEPTION_MEMORY_PARITY_ERROR = 0x08,
    LD_MODBUS_EXCEPTION_GATEWAY_PATH_UNAVAILABLE = 0x0A,
    LD_MODBUS_EXCEPTION_GATEWAY_TARGET_NO_RESPONSE = 0x0B
} ld_modbus_exception_t;

/** @brief Zero-copy view of a validated RTU or TCP ADU. */
typedef struct
{
    uint16_t transaction_id;
    uint8_t unit_id;
    const uint8_t *pdu;
    size_t pdu_length;
} ld_modbus_adu_view_t;

/** @brief Calculate the Modbus RTU CRC-16 for a byte sequence. */
uint16_t ld_modbus_crc16(const uint8_t *data, size_t length);

/**
 * @brief Encode a complete Modbus RTU ADU into caller-owned storage.
 * @return Explicit status; output_length is written only on success.
 */
ld_modbus_status_t ld_modbus_rtu_encode(uint8_t unit_id,
                                        const uint8_t *pdu,
                                        size_t pdu_length,
                                        uint8_t *output,
                                        size_t output_capacity,
                                        size_t *output_length);

/**
 * @brief Validate and view a complete Modbus RTU ADU without copying its PDU.
 * @note The returned view remains valid only while adu remains unchanged.
 */
ld_modbus_status_t ld_modbus_rtu_decode(const uint8_t *adu,
                                        size_t adu_length,
                                        ld_modbus_adu_view_t *view);

/** @brief Encode a complete Modbus TCP ADU with an MBAP header. */
ld_modbus_status_t ld_modbus_tcp_encode(uint16_t transaction_id,
                                        uint8_t unit_id,
                                        const uint8_t *pdu,
                                        size_t pdu_length,
                                        uint8_t *output,
                                        size_t output_capacity,
                                        size_t *output_length);

/** @brief Validate and view a complete Modbus TCP ADU without copying its PDU. */
ld_modbus_status_t ld_modbus_tcp_decode(const uint8_t *adu,
                                        size_t adu_length,
                                        ld_modbus_adu_view_t *view);

/** @brief Return a stable diagnostic string for a library status. */
const char *ld_modbus_status_string(ld_modbus_status_t status);

#ifdef __cplusplus
}
#endif

#endif
