#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>

/**
 * @brief Calculate 8-bit CRC checksum using lookup table.
 *
 * @note the mathmatical details are unknown.
 *	this is only for crc8 in robomaster referee system
 *
 * @param buff Pointer to the data buffer to be verified.
 * @param len  Length of the data buffer in bytes.
 * @return 8-bit CRC residual (checksum).
 */
uint8_t crc8_get_checksum(const uint8_t *buff, uint32_t len);

/**
 * @brief Calculate 16-bit CRC checksum using lookup table.
 *
 * using standard CRC-16/CCITT-FALSE, with
 * P(x) = x^16 + x^12 + x^5 + 1 (0x1021) and initial value 0xFFFF
 *
 * @param buff Pointer to the data buffer to be verified.
 * @param len  Length of the data buffer in bytes.
 * @return 16-bit CRC residual (checksum).
 */
uint16_t crc16_get_checksum(const uint8_t *buff, uint32_t len);

/**
 * @brief Generic mathematical bit-by-bit CRC calculation engine.
 *
 * Simulates the standard MSB-first polynomial long division over GF(2)
 * dynamically. Supports arbitrary widths up to 32 bits without a lookup table.
 *
 * @param buff      Pointer to the data buffer.
 * @param len       Length of the data buffer in bytes.
 * @param poly      The generator polynomial coefficients (omitting the highest x^w term).
 * @param crc_width The bit width of the CRC (e.g., 8, 16, 32).
 * @param init_val  The initial value of the CRC shift register.
 * @return	The calculated CRC residual mask-aligned to crc_width.
 */
uint32_t crc_checksum_generic(const uint8_t *buff, uint32_t len, uint32_t poly, int width,
			      uint32_t init_val);

#endif /* CRC_H_ */
