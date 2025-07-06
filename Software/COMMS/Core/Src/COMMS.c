/*
 * COMMS.c
 *
 *  Created on: Apr 18, 2025
 *      Author: mazen
 */

#include "COMMS.h"  // Include header for definitions and declarations

// ---------------------------------------------------------
// Function: compute_crc_ccitt
// Purpose:  Compute 16-bit CRC (CCITT) for AX.25 error detection
// ---------------------------------------------------------
static uint16_t compute_crc_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF; // Start with all bits set
    for (int i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8); // XOR byte into high byte of CRC
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)  // If highest bit is set
                crc = (crc << 1) ^ 0x1021; // Shift and XOR with polynomial
            else
                crc <<= 1; // Just shift
        }
    }
    return ~crc; // Return one's complement
}

// ---------------------------------------------------------
// Function: COMMS_SendAX25
// Purpose:  Construct and send an AX.25 frame over UART
// ---------------------------------------------------------
void COMMS_SendAX25(UART_HandleTypeDef *huart, const char *dest, const char *src, const uint8_t *payload, uint8_t len) {
    uint8_t frame[MAX_FRAME_SIZE];  // AX.25 frame buffer
    uint16_t i = 0;                 // Index for frame position

    frame[i++] = AX25_FLAG; // Start flag (0x7E)

    // Encode destination address (callsign shifted left by 1)
    for (int j = 0; j < 6; j++)
        frame[i++] = (j < strlen(dest)) ? (dest[j] << 1) : (' ' << 1);
    frame[i++] = (0 << 1) | 0x00; // SSID for dest, last address bit not set

    // Encode source address (callsign shifted left by 1)
    for (int j = 0; j < 6; j++)
        frame[i++] = (j < strlen(src)) ? (src[j] << 1) : (' ' << 1);
    frame[i++] = (0 << 1) | 0x01; // SSID for source, last address bit **set**

    frame[i++] = AX25_CONTROL; // Control field (0x03 = UI frame)
    frame[i++] = AX25_PID;     // Protocol ID (0xF0 = no layer 3)

    // Append payload (Info field)
    for (int j = 0; j < len; j++)
        frame[i++] = payload[j];

    // Compute CRC over everything except the flags
    uint16_t crc = compute_crc_ccitt(&frame[1], i - 1);
    frame[i++] = crc & 0xFF;        // CRC low byte
    frame[i++] = (crc >> 8) & 0xFF; // CRC high byte

    frame[i++] = AX25_FLAG; // End flag (0x7E)

    // Send the frame over UART
    HAL_UART_Transmit(huart, frame, i, HAL_MAX_DELAY);
}
