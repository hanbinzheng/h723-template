#ifndef BSP_FDCAN_H_
#define BSP_FDCAN_H_

#include "fdcan.h"
#include <stdbool.h>
#include <stdint.h>

struct can_inst;
typedef void (*can_callback)(struct can_inst *inst, uint8_t *rx_buff);

/* only classical can is supported, no fdcan */
enum can_type {
	CAN_STANDARD = 0,
	CAN_EXTENDED,
};

struct can_config {
	FDCAN_HandleTypeDef *hfdcan;
	enum can_type type;
	uint32_t rx_id;	       /* receive can id, used to configure filter */
	uint32_t mask;	       /* receive filter mask */
	can_callback callback; /* receive callback function */
	uint32_t tx_id;	       /* transmit can id, used for can_transmit */
};

/**
 * @brief can_init() - Initialize all FDCAN instances and start reception.
 *
 * This function configures the global filter, activates the receive FIFO0
 * interrupt, and starts all FDCAN peripherals.
 *
 * The global filter is set to reject all remote frames and accept all unmatched data frame
 *
 * @param config pointer to the configuration
 * @return: pointer to a can instance on success, NULL if any filure
 */
struct can_inst *can_register(const struct can_config *config);

/**
 * Function to complete the final configuration and start fdcan peripheral
 *
 * configure the global filter, open the rx interrupt and start the fdcan peripheral
 * @note this function should be called after all configuration by can_register()
 *
 * @return: HAL_OK if successful and HAL_ERROR otherwise.
 */
HAL_StatusTypeDef can_start(void);

/**
 * @brief can_transmit() - Transmit a CAN message
 *
 * Transmits a classical CAN message using the specified FDCAN peripheral.
 * The data buffer must contain exactly 8 bytes
 *
 * @param inst: Pointer to can instance
 * @param buff: Pointer to 8-byte data buffer to transmit
 *
 * @return HAL_OK on success, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef can_transmit(const struct can_inst *inst, const uint8_t *buff);

#endif /* BSP_FDCAN_H_ */