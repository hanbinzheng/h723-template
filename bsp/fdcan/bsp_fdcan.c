#include "bsp_fdcan.h"
#include "hash.h"
#include <assert.h>

#define CAN_BUS_NUM 3	  /* 3 FDCAN peripherals in total */
#define CAN_STD_MAX 16	  /* as configured in cubemx, for a signle bus */
#define CAN_EXT_MAX 8	  /* as configured in cubemx, for a single bus */
#define CAN_DATA_LENGTH 8 /* classical CAN, data frame 8 byte */

struct can_inst {
	FDCAN_HandleTypeDef *hfdcan;
	uint32_t rx_id;
	can_callback callback;
	uint32_t tx_id;
	FDCAN_TxHeaderTypeDef tx_header;
};

/* physical CAN peripheral: fdcan_instance */
struct can_bus {
	uint8_t bus_idx; /* which can bus */
	FDCAN_HandleTypeDef *hfdcan;
	struct hash_table table; /* can id -> callback */

	/* standard and extended instance */
	uint8_t idx_std;
	uint8_t idx_ext;
	struct can_inst inst_std[CAN_STD_MAX];
	struct can_inst inst_ext[CAN_EXT_MAX];
};

/* static variables and helper functions */
static struct can_bus can_bus[CAN_BUS_NUM] = {0};
static HAL_StatusTypeDef add_filter(struct can_bus *canbus, const struct can_config *config);
static void config_inst_elements(const struct can_config *config, struct can_inst *inst);

__ALWAYS_INLINE
static struct can_bus *get_canbus(FDCAN_HandleTypeDef *hfdcan)
{
	for (uint8_t i = 0; i < CAN_BUS_NUM; i++) {
		if (can_bus[i].hfdcan == hfdcan)
			return (can_bus + i);
	}

	return NULL; /* fails */
}

__ALWAYS_INLINE
static struct can_bus *update_canbus(FDCAN_HandleTypeDef *hfdcan)
{
	for (uint8_t i = 0; i < CAN_BUS_NUM; i++) {
		struct can_bus *canbus = can_bus + i;
		if (canbus->hfdcan == NULL) {
			canbus->bus_idx = i;
			canbus->hfdcan = hfdcan;
			hash_init(&(canbus->table));
			return canbus;
		}
	}

	return NULL; /* fails */
}

/* The recommended initialization order is:
 * 1. Configure individual filters (HAL_FDCAN_ConfigFilter)
 * 2. Configure the global filter (HAL_FDCAN_ConfigGlobalFilter)
 * 3. Activate reception interrupt (HAL_FDCAN_ActivateNotification)
 * 4. Start the FDCAN peripheral (HAL_FDCAN_Start)
 *
 * Reasons:
 * HAL_FDCAN_ConfigGlobalFilter requires hfdcan->State to be
 * HAL_FDCAN_STATE_READY, while HAL_FDCAN_Start sets state to
 * HAL_FDCAN_STATE_BUSY. HAL_FDCAN_ConfigFilter and
 * HAL_FDCAN_ActivateNotification accept both states.
 */
struct can_inst *can_register(const struct can_config *config)
{
	/* check any error */
	assert(config != NULL && config->hfdcan != NULL);
	struct can_bus *canbus = get_canbus(config->hfdcan);
	if (canbus == NULL) {
		canbus = update_canbus(config->hfdcan);
		if (canbus == NULL)
			return NULL; /* hfdcan error */
	}

	/* get can instance */
	struct can_inst *inst = NULL;
	switch (config->type) {
	case CAN_STANDARD:
		if (canbus->idx_std >= CAN_STD_MAX)
			return NULL; /* exceed the max number */
		for (uint8_t i = 0; i < canbus->idx_std; i++) {
			if (canbus->inst_std[i].rx_id == config->rx_id)
				return NULL; /* check repetition */
		}
		inst = canbus->inst_std + canbus->idx_std;
		break;
	case CAN_EXTENDED:
		if (canbus->idx_ext >= CAN_EXT_MAX)
			return NULL;
		for (uint8_t i = 0; i < canbus->idx_ext; i++) {
			if (canbus->inst_ext[i].rx_id == config->rx_id)
				return NULL;
		}
		inst = canbus->inst_ext + canbus->idx_ext;
		break;
	}

	/* add filter and other configuration */
	if (add_filter(canbus, config) != HAL_OK) {
		return NULL;
	}
	config_inst_elements(config, inst);
	hash_insert(&(canbus->table), inst->rx_id, (uint32_t)inst); /* no need to check */

	/* update index */
	if (config->type == CAN_STANDARD)
		canbus->idx_std++;
	else
		canbus->idx_ext++;

	return inst;
}

HAL_StatusTypeDef can_start(void)
{
	HAL_StatusTypeDef ret = HAL_OK;

	for (int i = 0; i < CAN_BUS_NUM; i++) {
		struct can_bus *canbus = can_bus + i;
		if (canbus->hfdcan == NULL) {
			continue;
		}

		/* configure the global filter */
		if (HAL_FDCAN_ConfigGlobalFilter(canbus->hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0,
						 FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE,
						 FDCAN_REJECT_REMOTE) != HAL_OK)
			ret = HAL_ERROR;
		/* activate reception interrupt */
		if (HAL_FDCAN_ActivateNotification(canbus->hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
						   0) != HAL_OK)
			ret = HAL_ERROR;
		/* start the can peripheral */
		if (HAL_FDCAN_Start(canbus->hfdcan) != HAL_OK)
			ret = HAL_ERROR;
	}

	return ret;
}

HAL_StatusTypeDef can_transmit(const struct can_inst *inst, const uint8_t *buff)
{
	assert(inst != NULL && buff != NULL);

	if (HAL_FDCAN_GetTxFifoFreeLevel(inst->hfdcan) == 0)
		return HAL_BUSY; /* whether FIFO is full */

	return HAL_FDCAN_AddMessageToTxFifoQ(inst->hfdcan, &(inst->tx_header), buff);
}

/**
 * HAL_FDCAN_RxFifo0Callback() - FDCAN RxFIFO0 callback (weak function override)
 *
 * This callback is invoked by the HAL when a new message arrives in RxFIFO0.
 * It retrieves the received message using HAL_FDCAN_GetRxMessage and forwards
 * it to the user-registered callback for processing*
 *
 * @param hfdcan Pointer to FDCAN handle
 * @param RxFifo0ITs RxFIFO0 interrupt status
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
		FDCAN_RxHeaderTypeDef header;
		uint8_t rx_buff[CAN_DATA_LENGTH];
		struct can_bus *canbus = get_canbus(hfdcan);
		if (canbus == NULL)
			return;

		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, rx_buff) != HAL_OK)
			return; /* fails to get message */

		struct can_inst *inst = NULL;
		if (hash_lookup(&(canbus->table), header.Identifier, (uint32_t *)&inst)) {
			if (inst != NULL) {
				inst->callback(inst, rx_buff);
			}
		}
	}
}

/**
 * add_filter() - Helper function to configure a single filter
 *
 * Configure a filter for a FDCAN instance. ( in Message RAM )
 * All mesages matching are reported to RxFIFO0.
 *
 * @param canbus which canbus, assumed correct, no check
 * @param config can instance configureation struct
 * @return: HAL_OK if successful and HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef add_filter(struct can_bus *canbus, const struct can_config *config)
{
	FDCAN_FilterTypeDef filter;

	/* STM32H7 FDCAN hardware filter index handling
	 *
	 * Standard and extended ID filters are stored in separate RAM areas.
	 * This means a standard filter and an extended filter can use
	 * the same FilterIndex value without conflict.
	 */
	switch (config->type) {
	case CAN_STANDARD:
		filter.IdType = FDCAN_STANDARD_ID;
		filter.FilterIndex = canbus->idx_std;
		break;
	case CAN_EXTENDED:
		filter.IdType = FDCAN_EXTENDED_ID;
		filter.FilterIndex = canbus->idx_ext;
	}

	/*
	 * RxBufferIndex and IsCalibrationMsg are omitted, that is because
	 * both parameters will be ignored if FilterConfig is different from
	 * FDCAN_FILTER_TO_BUFFER.
	 */
	filter.FilterType = FDCAN_FILTER_MASK;	       /* Matching: FilterID1 & FilterID2 */
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; /* Always reports to RxFIFO0 */
	filter.FilterID1 = config->rx_id;
	filter.FilterID2 = config->mask;

	if (HAL_FDCAN_ConfigFilter(config->hfdcan, &filter) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}

static void config_inst_elements(const struct can_config *config, struct can_inst *inst)
{
	/* basic copy */
	inst->hfdcan = config->hfdcan;
	inst->rx_id = config->rx_id;
	inst->tx_id = config->tx_id;
	inst->callback = config->callback;

	/* configure the transmit header */
	FDCAN_TxHeaderTypeDef tx_header = {
	    .Identifier = inst->tx_id,
	    .IdType = (config->type == CAN_STANDARD) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID,
	    .TxFrameType = FDCAN_DATA_FRAME, /* data frame only */
	    .DataLength = FDCAN_DLC_BYTES_8, /* classical CAN */
	    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
	    .BitRateSwitch = FDCAN_BRS_OFF,
	    .FDFormat = FDCAN_CLASSIC_CAN,
	    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
	    .MessageMarker = 0,
	};
	inst->tx_header = tx_header;
}
