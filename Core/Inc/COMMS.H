/*
 * COMMS.H
 *
 *  Created on: Apr 18, 2025
 *      Author: mazen
 */

#ifndef INC_COMMS_H_
#define INC_COMMS_H_

#include "stm32f1xx.h"
#include <string.h>

#define AX25_CONTROL     0x03    // UI-frame control field
#define AX25_FLAG        0x7E    // Frame boundary flag
#define AX25_PID         0xF0    // No Layer 3 protocol
#define MAX_FRAME_SIZE   275     // Maximum AX.25 frame size
#define MAX_PAYLOAD_SIZE 255     // Maximum payload size

#ifdef __cplusplus
extern "C" {
#endif

void COMMS_SendAX25(UART_HandleTypeDef *huart, const char *dest, const char *src, const uint8_t *payload, uint8_t len);
int  COMMS_ReceiveAX25(UART_HandleTypeDef *huart, char *dest, char *src, uint8_t *payload, uint16_t max_payload_len);
uint16_t compute_crc_ccitt(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* INC_COMMS_H_ */
