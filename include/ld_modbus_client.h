/**
 * @file ld_modbus_client.h
 * @brief Stateless Modbus client PDU builders and response parsers.
 */

#ifndef LD_MODBUS_CLIENT_H
#define LD_MODBUS_CLIENT_H

#include "ld_modbus.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Build function 01, 02, 03, or 04 read request PDU. */
ld_modbus_status_t ld_modbus_client_build_read_request(uint8_t function,
                                                       uint16_t address,
                                                       uint16_t quantity,
                                                       uint8_t *pdu,
                                                       size_t pdu_capacity,
                                                       size_t *pdu_length);

/** @brief Build function 05 write-single-coil request PDU. */
ld_modbus_status_t ld_modbus_client_build_write_single_coil(uint16_t address,
                                                            uint8_t value,
                                                            uint8_t *pdu,
                                                            size_t pdu_capacity,
                                                            size_t *pdu_length);

/** @brief Build function 06 write-single-register request PDU. */
ld_modbus_status_t ld_modbus_client_build_write_single_register(uint16_t address,
                                                                uint16_t value,
                                                                uint8_t *pdu,
                                                                size_t pdu_capacity,
                                                                size_t *pdu_length);

/** @brief Build function 0F request from one-byte-per-coil input values. */
ld_modbus_status_t ld_modbus_client_build_write_multiple_coils(uint16_t address,
                                                               const uint8_t *values,
                                                               uint16_t quantity,
                                                               uint8_t *pdu,
                                                               size_t pdu_capacity,
                                                               size_t *pdu_length);

/** @brief Build function 10 request from host-endian register values. */
ld_modbus_status_t ld_modbus_client_build_write_multiple_registers(
    uint16_t address,
    const uint16_t *values,
    uint16_t quantity,
    uint8_t *pdu,
    size_t pdu_capacity,
    size_t *pdu_length);

/** @brief Build function 16 mask-write-register request PDU. */
ld_modbus_status_t ld_modbus_client_build_mask_write_register(
    uint16_t address,
    uint16_t and_mask,
    uint16_t or_mask,
    uint8_t *pdu,
    size_t pdu_capacity,
    size_t *pdu_length);

/** @brief Build function 17 write/read-multiple-registers request PDU. */
ld_modbus_status_t ld_modbus_client_build_write_read_multiple_registers(
    uint16_t read_address,
    uint16_t read_quantity,
    uint16_t write_address,
    const uint16_t *write_values,
    uint16_t write_quantity,
    uint8_t *pdu,
    size_t pdu_capacity,
    size_t *pdu_length);

/** @brief Parse a 01/02 response into one-byte-per-bit destination storage. */
ld_modbus_status_t ld_modbus_client_parse_read_bits_response(
    uint8_t expected_function,
    uint16_t expected_quantity,
    const uint8_t *pdu,
    size_t pdu_length,
    uint8_t *values,
    size_t values_capacity,
    uint8_t *exception_code);

/** @brief Parse a 03/04/17 response into host-endian register storage. */
ld_modbus_status_t ld_modbus_client_parse_read_registers_response(
    uint8_t expected_function,
    uint16_t expected_quantity,
    const uint8_t *pdu,
    size_t pdu_length,
    uint16_t *values,
    size_t values_capacity,
    uint8_t *exception_code);

/** @brief Validate the fixed echo response used by functions 05, 06, 0F, and 10. */
ld_modbus_status_t ld_modbus_client_parse_write_response(uint8_t expected_function,
                                                         uint16_t expected_address,
                                                         uint16_t expected_value_or_quantity,
                                                         const uint8_t *pdu,
                                                         size_t pdu_length,
                                                         uint8_t *exception_code);

/** @brief Validate the seven-byte echo response used by function 16. */
ld_modbus_status_t ld_modbus_client_parse_mask_write_response(
    uint16_t expected_address,
    uint16_t expected_and_mask,
    uint16_t expected_or_mask,
    const uint8_t *pdu,
    size_t pdu_length,
    uint8_t *exception_code);

#ifdef __cplusplus
}
#endif

#endif
