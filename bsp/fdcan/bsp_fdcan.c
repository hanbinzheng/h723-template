#include "bsp_fdcan.h"
#include <assert.h>

#define CAN_BUS_NUM 3	      /* 3 FDCAN peripherals in total */
#define CAN_DATA_LENGTH 8     /* classical CAN, data frame 8 byte */
#define CAN_TXINST_MAX 8      /* actually unlimited, but 8 is enough and simple */
#define CAN_RXINST_STD_MAX 16 /* as configured in cubemx, for a signle bus */
#define CAN_RXINST_EXT_MAX 8  /* as configured in cubemx, for a single bus */

/* check FDCAN_RxHeaderTypeDef.IsFilterMatchingFrame in stm32h7xx_hal_fdcan.h */
#define FILTER_MATCHING 0
#define FILTER_NOT_MATCHING 1

#define IDX_IN_RANGE(id_type, idx)                                                                 \
	((id_type) == FDCAN_STANDARD_ID ? (idx) < CAN_RXINST_STD_MAX : (idx) < CAN_RXINST_EXT_MAX)
#define GET_INST_BUFF(canbus, id_type)                                                             \
	((id_type) == FDCAN_STANDARD_ID ? (canbus)->rxinst_std : (canbus)->rxinst_ext)

struct can_rx_inst {
	FDCAN_HandleTypeDef *hfdcan;
	uint32_t id;
	can_rx_callback callback;
};

struct can_tx_inst {
	FDCAN_HandleTypeDef *hfdcan;
	FDCAN_TxHeaderTypeDef header;
	// uint32_t type; /* IdType in FDCAN_TxHeaderTypeDef */
	// uint32_t id;   /* Identifier in FDCAN_TxHeaderTypeDef */
};

/* physical CAN peripheral: fdcan_instance */
struct can_bus {
	uint8_t bus_idx; /* which canbus */
	FDCAN_HandleTypeDef *hfdcan;

	/* rx inst: standard and extended */
	uint8_t rxidx_std;
	uint8_t rxidx_ext;
	struct can_rx_inst rxinst_std[CAN_RXINST_STD_MAX];
	struct can_rx_inst rxinst_ext[CAN_RXINST_EXT_MAX];

	/* tx inst */
	uint8_t txidx;
	struct can_tx_inst txinst[CAN_TXINST_MAX];
};

/* static variables and helper functions */
static struct can_bus can_bus[CAN_BUS_NUM] = {0};
static HAL_StatusTypeDef add_filter(struct can_bus *canbus, const struct can_rx_config *config);
static struct can_rx_inst *get_rx_inst(struct can_bus *canbus, const struct can_rx_config *config);

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
static struct can_bus *config_canbus(FDCAN_HandleTypeDef *hfdcan)
{
	for (uint8_t i = 0; i < CAN_BUS_NUM; i++) {
		struct can_bus *canbus = can_bus + i;
		if (canbus->hfdcan == NULL) {
			canbus->bus_idx = i;
			canbus->hfdcan = hfdcan;
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
struct can_rx_inst *can_register_rx(const struct can_rx_config *config)
{
	/* check if any error */
	assert(config != NULL && config->hfdcan != NULL);
	struct can_bus *canbus = get_canbus(config->hfdcan);
	if (canbus == NULL) {
		canbus = config_canbus(config->hfdcan);
		if (canbus == NULL) {
			return NULL; /* hfdcan error */
		}
	}

	/* get rx inst */
	struct can_rx_inst *inst = get_rx_inst(canbus, config);
	if (inst == NULL) {
		return NULL;
	}

	/* other configuration */
	if (add_filter(canbus, config) != HAL_OK) {
		return NULL;
	}
	inst->id = config->id;
	inst->hfdcan = config->hfdcan;
	inst->callback = config->callback;

	/* update canbus index */
	if (config->type == CAN_STANDARD) {
		canbus->rxidx_std++;
	} else {
		canbus->rxidx_ext++;
	}

	return inst;
}

struct can_tx_inst *can_register_tx(const struct can_tx_config *config)
{
	/* check if any error */
	assert(config != NULL && config->hfdcan != NULL);
	struct can_bus *canbus = get_canbus(config->hfdcan);
	if (canbus == NULL) {
		canbus = config_canbus(config->hfdcan);
		if (canbus == NULL) {
			return NULL; /* hfdcan error */
		}
	}

	/* get tx inst */
	struct can_tx_inst *inst = NULL;
	if (canbus->txidx >= CAN_TXINST_MAX) {
		return NULL;
	} else {
		for (uint8_t i = 0; i < canbus->txidx; i++) {
			struct can_tx_inst *tmp = canbus->txinst + i;
			if (tmp->header.IdType == config->type &&
			    tmp->header.Identifier == config->id) {
				return tmp; /* check repetition */
			}
		}
		inst = canbus->txinst + canbus->txidx;
		canbus->txidx++;
	}

	/* configure data */
	FDCAN_TxHeaderTypeDef header = {
	    .Identifier = config->id,
	    .IdType = config->type,
	    .TxFrameType = FDCAN_DATA_FRAME, /* data frame only */
	    .DataLength = FDCAN_DLC_BYTES_8, /* classical CAN */
	    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
	    .BitRateSwitch = FDCAN_BRS_OFF,
	    .FDFormat = FDCAN_CLASSIC_CAN,
	    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
	    .MessageMarker = 0,
	};
	inst->hfdcan = config->hfdcan;
	inst->header = header;

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

		/* the following three functions only returns HAL_OK or HAL_ERROR */

		/* configure the global filter */
		if (HAL_FDCAN_ConfigGlobalFilter(canbus->hfdcan, FDCAN_REJECT, FDCAN_REJECT,
						 FDCAN_REJECT_REMOTE,
						 FDCAN_REJECT_REMOTE) != HAL_OK) {
			ret = HAL_ERROR;
		}
		/* activate reception interrupt */
		if (HAL_FDCAN_ActivateNotification(canbus->hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
						   0) != HAL_OK) {
			ret = HAL_ERROR;
		}
		/* start the can peripheral */
		if (HAL_FDCAN_Start(canbus->hfdcan) != HAL_OK) {
			ret = HAL_ERROR;
		}
	}

	return ret;
}

HAL_StatusTypeDef can_transmit(const struct can_tx_inst *inst, const uint8_t *buff)
{
	assert(inst != NULL && buff != NULL && inst->hfdcan != NULL);

	if (HAL_FDCAN_GetTxFifoFreeLevel(inst->hfdcan) == 0) {
		return HAL_BUSY; /* whether FIFO is full */
	}

	/* configure tx inst elements */
	return HAL_FDCAN_AddMessageToTxFifoQ(inst->hfdcan, &(inst->header), buff);
}

void can_set_tx_id(struct can_tx_inst *inst, uint32_t id)
{
	assert(inst != NULL && inst->hfdcan != NULL);
	inst->header.Identifier = id;
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
		uint8_t buff[CAN_DATA_LENGTH];
		struct can_bus *canbus = get_canbus(hfdcan);
		if (canbus == NULL)
			return;

		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, buff) != HAL_OK) {
			return; /* fails to get message */
		}

		/* get rx inst */
		struct can_rx_inst *inst = NULL;
		if (header.IsFilterMatchingFrame == FILTER_MATCHING) {
			struct can_rx_inst *inst_buff = GET_INST_BUFF(canbus, header.IdType);
			if (IDX_IN_RANGE(header.IdType, header.FilterIndex)) {
				inst = inst_buff + header.FilterIndex;
			}
		}

		if (inst != NULL && inst->callback != NULL) {
			inst->callback(inst, buff);
		}
	}
}

/* assume that canbus and config are all valid */
static struct can_rx_inst *get_rx_inst(struct can_bus *canbus, const struct can_rx_config *config)
{
	struct can_rx_inst *inst = NULL;

	if (config->type == CAN_STANDARD) {
		if (canbus->rxidx_std >= CAN_RXINST_STD_MAX) {
			return NULL; /* exceed maximum number */
		}
		for (uint8_t i = 0; i < canbus->rxidx_std; i++) {
			if (canbus->rxinst_std[i].id == config->id) {
				return NULL; /* check repetition */
			}
		}
		inst = canbus->rxinst_std + canbus->rxidx_std;
	} else {
		if (canbus->rxidx_ext >= CAN_RXINST_EXT_MAX) {
			return NULL;
		}
		for (uint8_t i = 0; i < canbus->rxidx_ext; i++) {
			if (canbus->rxinst_ext[i].id == config->id) {
				return NULL;
			}
		}
		inst = canbus->rxinst_ext + canbus->rxidx_ext;
	}

	return inst;
}

/**
 * add_filter() - Helper function to configure a single filter
 *
 * Configure a filter for a FDCAN instance. ( in Message RAM )
 * All mesages matching are reported to RxFIFO0.
 *
 * @param canbus which canbus, assumed correct, no check
 * @param config rx instance configureation struct
 * @return: HAL_OK if successful and HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef add_filter(struct can_bus *canbus, const struct can_rx_config *config)
{
	FDCAN_FilterTypeDef filter = {0};

	/* STM32H7 FDCAN hardware filter index handling
	 *
	 * Standard and extended ID filters are stored in separate RAM areas.
	 * This means a standard filter and an extended filter can use
	 * the same FilterIndex value without conflict.
	 */
	if (config->type == CAN_STANDARD) {
		filter.IdType = FDCAN_STANDARD_ID;
		filter.FilterIndex = canbus->rxidx_std;
	} else {
		filter.IdType = FDCAN_EXTENDED_ID;
		filter.FilterIndex = canbus->rxidx_ext;
	}

	/*
	 * RxBufferIndex and IsCalibrationMsg are omitted, that is because
	 * both parameters will be ignored if FilterConfig is different from
	 * FDCAN_FILTER_TO_BUFFER.
	 */
	filter.FilterType = FDCAN_FILTER_MASK;	       /* Matching: FilterID1 & FilterID2 */
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; /* Always reports to RxFIFO0 */
	filter.FilterID1 = config->id;
	filter.FilterID2 = config->mask;

	return HAL_FDCAN_ConfigFilter(config->hfdcan, &filter);
}
