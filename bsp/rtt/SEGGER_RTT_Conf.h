/*********************************************************************
*                   (c) SEGGER Microcontroller GmbH                  *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************
*                                                                    *
*        SEGGER RTT * Real Time Transfer for embedded targets        *
*                  https://github.com/SEGGERMicro/RTT                *
*                                                                    *
**********************************************************************

---------------------------END-OF-HEADER------------------------------
Purpose : User configuration file for RTT.
	  For available configuration,
	  refer to SEGGER_RTT_ConfDefaults.h.

----------------------------------------------------------------------
*/

#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

/*********************************************************************
 *
 *       Defines, configurable
 *
 **********************************************************************
 */

#define SEGGER_RTT_MAX_NUM_UP_BUFFERS (3)		      /* default */
#define SEGGER_RTT_MAX_NUM_DOWN_BUFFERS (3)		      /* default */
#define BUFFER_SIZE_UP (2048)				      /* default: 1024 */
#define BUFFER_SIZE_DOWN (16)				      /* default */
#define SEGGER_RTT_MODE_DEFAULT SEGGER_RTT_MODE_NO_BLOCK_SKIP /* default */
#define SEGGER_RTT_PRINTF_BUFFER_SIZE (256u)		      /* default: 64 */
#define SEGGER_RTT_MEMCPY_USE_BYTELOOP (0)		      /* default */
#define SEGGER_RTT_MAX_INTERRUPT_PRIORITY (0x20)	      /* default */

#endif
/*************************** End of file ****************************/
