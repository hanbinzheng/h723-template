#ifndef F103_H_
#define F103_H_

#include "bsp_usart.h"

void f103_init(void);
HAL_StatusTypeDef f103_send(uint8_t *buff, uint16_t len);

#endif /* F103_H_ */
