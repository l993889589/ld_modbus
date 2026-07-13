/**
 * @file ld_modbus_server.h
 * @brief Static application-owned register map and server PDU processing.
 */

#ifndef LD_MODBUS_SERVER_H
#define LD_MODBUS_SERVER_H

#include "ld_modbus.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Application-owned static Modbus address tables and base addresses. */
typedef struct
{
    uint8_t *coils;
    uint16_t coils_start;
    uint16_t coils_count;

    uint8_t *discrete_inputs;
    uint16_t discrete_inputs_start;
    uint16_t discrete_inputs_count;

    uint16_t *holding_registers;
    uint16_t holding_registers_start;
    uint16_t holding_registers_count;

    uint16_t *input_registers;
    uint16_t input_registers_start;
    uint16_t input_registers_count;
} ld_modbus_server_map_t;

/**
 * @brief Read one coil from an application-owned server map.
 * @param map Server map initialized by the application.
 * @param address Zero-based Modbus data address, including the configured table start.
 * @param value Receives a normalized value of 0 or 1 on success.
 * @return OK, INVALID_ARGUMENT for null arguments, or RANGE_ERROR when unmapped.
 * @note The caller owns synchronization if protocol processing can access the map concurrently.
 */
ld_modbus_status_t ld_modbus_server_map_read_coil(const ld_modbus_server_map_t *map,
                                                  uint16_t address,
                                                  uint8_t *value);

/**
 * @brief Write one coil in an application-owned server map.
 * @param map Server map initialized by the application.
 * @param address Zero-based Modbus data address, including the configured table start.
 * @param value Any nonzero value is stored as 1; zero is stored as 0.
 * @return OK, INVALID_ARGUMENT for a null map, or RANGE_ERROR when unmapped.
 */
ld_modbus_status_t ld_modbus_server_map_write_coil(ld_modbus_server_map_t *map,
                                                   uint16_t address,
                                                   uint8_t value);

/**
 * @brief Read one discrete input from an application-owned server map.
 * @param map Server map initialized by the application.
 * @param address Zero-based Modbus data address, including the configured table start.
 * @param value Receives a normalized value of 0 or 1 on success.
 * @return OK, INVALID_ARGUMENT for null arguments, or RANGE_ERROR when unmapped.
 */
ld_modbus_status_t ld_modbus_server_map_read_discrete_input(
    const ld_modbus_server_map_t *map,
    uint16_t address,
    uint8_t *value);

/**
 * @brief Update one discrete input from the local application or sensor layer.
 * @param map Server map initialized by the application.
 * @param address Zero-based Modbus data address, including the configured table start.
 * @param value Any nonzero value is stored as 1; zero is stored as 0.
 * @return OK, INVALID_ARGUMENT for a null map, or RANGE_ERROR when unmapped.
 * @note This does not make discrete inputs writable by a remote Modbus client.
 */
ld_modbus_status_t ld_modbus_server_map_set_discrete_input(ld_modbus_server_map_t *map,
                                                           uint16_t address,
                                                           uint8_t value);

/**
 * @brief Read one holding register from an application-owned server map.
 * @param map Server map initialized by the application.
 * @param address Zero-based Modbus data address, including the configured table start.
 * @param value Receives the register value on success.
 * @return OK, INVALID_ARGUMENT for null arguments, or RANGE_ERROR when unmapped.
 */
ld_modbus_status_t ld_modbus_server_map_read_holding_register(
    const ld_modbus_server_map_t *map,
    uint16_t address,
    uint16_t *value);

/**
 * @brief Write one holding register in an application-owned server map.
 * @param map Server map initialized by the application.
 * @param address Zero-based Modbus data address, including the configured table start.
 * @param value Register value to store.
 * @return OK, INVALID_ARGUMENT for a null map, or RANGE_ERROR when unmapped.
 */
ld_modbus_status_t ld_modbus_server_map_write_holding_register(
    ld_modbus_server_map_t *map,
    uint16_t address,
    uint16_t value);

/**
 * @brief Read one input register from an application-owned server map.
 * @param map Server map initialized by the application.
 * @param address Zero-based Modbus data address, including the configured table start.
 * @param value Receives the register value on success.
 * @return OK, INVALID_ARGUMENT for null arguments, or RANGE_ERROR when unmapped.
 */
ld_modbus_status_t ld_modbus_server_map_read_input_register(
    const ld_modbus_server_map_t *map,
    uint16_t address,
    uint16_t *value);

/**
 * @brief Update one input register from the local application or sensor layer.
 * @param map Server map initialized by the application.
 * @param address Zero-based Modbus data address, including the configured table start.
 * @param value Register value to store.
 * @return OK, INVALID_ARGUMENT for a null map, or RANGE_ERROR when unmapped.
 * @note This does not make input registers writable by a remote Modbus client.
 */
ld_modbus_status_t ld_modbus_server_map_set_input_register(ld_modbus_server_map_t *map,
                                                           uint16_t address,
                                                           uint16_t value);

/** @brief Observable result of processing one complete server request ADU. */
typedef enum
{
    LD_MODBUS_SERVER_ACTION_IGNORED = 0,
    LD_MODBUS_SERVER_ACTION_REPLY,
    LD_MODBUS_SERVER_ACTION_BROADCAST_APPLIED
} ld_modbus_server_action_t;

/**
 * @brief Process one validated request PDU and build its response PDU.
 *
 * Protocol address/value errors are returned as valid Modbus exception PDUs
 * with LD_MODBUS_STATUS_OK. Malformed input and invalid C arguments return a
 * library error and do not produce a response.
 */
ld_modbus_status_t ld_modbus_server_process_pdu(const ld_modbus_server_map_t *map,
                                                const uint8_t *request_pdu,
                                                size_t request_length,
                                                uint8_t *response_pdu,
                                                size_t response_capacity,
                                                size_t *response_length);

/**
 * @brief Validate and process one complete RTU request ADU.
 * @param action Reports ignored traffic, a reply, or an applied broadcast.
 * @note Request and response storage must not overlap.
 */
ld_modbus_status_t ld_modbus_server_process_rtu_adu(
    const ld_modbus_server_map_t *map,
    uint8_t local_unit_id,
    const uint8_t *request_adu,
    size_t request_length,
    uint8_t *response_adu,
    size_t response_capacity,
    size_t *response_length,
    ld_modbus_server_action_t *action);

/**
 * @brief Validate and process one complete TCP request ADU.
 * @note The response preserves transaction and unit identifiers.
 * @note Request and response storage must not overlap.
 */
ld_modbus_status_t ld_modbus_server_process_tcp_adu(
    const ld_modbus_server_map_t *map,
    const uint8_t *request_adu,
    size_t request_length,
    uint8_t *response_adu,
    size_t response_capacity,
    size_t *response_length);

#ifdef __cplusplus
}
#endif

#endif
