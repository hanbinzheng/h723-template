#ifndef BSP_FDCAN_H_
#define BSP_FDCAN_H_

#include "fdcan.h"
#include <stdbool.h>
#include <stdint.h>

struct can_rx_inst;
struct can_tx_inst;
typedef void (*can_rx_callback)(struct can_rx_inst *inst, uint8_t *buff);

/* only classical can is supported, no fdcan */
enum can_type {
	CAN_STANDARD = FDCAN_STANDARD_ID,
	CAN_EXTENDED = FDCAN_EXTENDED_ID,
};

struct can_rx_config {
	FDCAN_HandleTypeDef *hfdcan;
	enum can_type type;
	uint32_t id;
	uint32_t mask;
	can_rx_callback callback;
};

struct can_tx_config {
	FDCAN_HandleTypeDef *hfdcan;
	enum can_type type;
	uint32_t id;
};

/**
 * @brief can_register_rx() - Initialize a rx FDCAN instance filter.
 *
 * @param config pointer to the rx configuration
 * @return: pointer to a can instance on success, NULL if any filure
 */
struct can_rx_inst *can_register_rx(const struct can_rx_config *config);

/**
 * @brief can_tx_register() - Initialize a tx FDCAN instance.
 *
 * @param config pointer to the rx configuration
 * @return: pointer to a can instance on success, NULL if any filure
 */
struct can_tx_inst *can_register_tx(const struct can_tx_config *config);

/**
 * @brief Function to complete the final configuration and start fdcan peripheral
 *
 * configure the global filter, open the rx interrupt and start the fdcan peripheral
 * @note this function should be called after all configuration by can_rx_register()
 *
 * @return: HAL_OK if successful and HAL_ERROR otherwise.
 */
HAL_StatusTypeDef can_start(void);

/**
 * @brief combine user data to a can instance
 *
 * @param inst can rx inst
 * @param data pointer to the user data
 */
void can_set_user_data(struct can_rx_inst *inst, const void *user_data);

/**
 * @brief get user data
 *
 * @param inst can rx inst
 * @return pointer to the user data
 */
void *can_get_user_data(const struct can_rx_inst *inst);

/*isnt*
 * @brief set the tx id
 *
 * @param inst: Pointer to can tx instance
 * @param id: new id
 */
void can_set_tx_id(struct can_tx_inst *inst, uint32_t id);

/**
 * @brief can_transmit() - Transmit a CAN message
 *
 * Transmits a classical CAN message using the specified FDCAN peripheral.
 * The data buffer must contain exactly 8 bytes
 *
 * @param inst: Pointer to can tx instance
 * @param buff: Pointer to 8-byte data buffer to transmit
 *
 * @return HAL_OK on success, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef can_transmit(const struct can_tx_inst *inst, const uint8_t *buff);

#endif /* BSP_FDCAN_H_ */
