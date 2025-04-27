/*
 * COMMS.H
 *
 *  Created on: Apr 18, 2025
 *      Author: mazen
 */

#ifndef INC_COMMS_H_  // Header guard to prevent multiple inclusion
#define INC_COMMS_H_

#include "stm32f1xx.h"   // STM32 HAL core definitions
#include <string.h>      // Standard C string functions (e.g., strlen)

#define AX25_CONTROL     0x03    // Control field for UI frame (Unnumbered Information)
#define AX25_FLAG        0x7E    // Flag byte used to mark start and end of AX.25 frame
#define AX25_PID         0xF0    // Protocol Identifier (no layer 3 protocol)
#define MAX_FRAME_SIZE   2320    // Maximum allowed frame size (larger to accommodate full AX.25 frames)


void COMMS_SendAX25(UART_HandleTypeDef *huart, const char *dest, const char *src, const uint8_t *payload, uint8_t len);
// Function to build a full AX.25 frame into the given buffer

#endif /* INC_COMMS_H_ */  // End of header guard
