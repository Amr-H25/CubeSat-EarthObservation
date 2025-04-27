// COMMS.c
#include "COMMS.h"

// ---------------------------------------------------------
// Function: compute_crc_ccitt
// Purpose:  Calculate 16-bit CRC (Cyclic Redundancy Check)
//           using the CCITT polynomial 0x1021
// Inputs:
//   - data: pointer to the input data array
//   - len: number of bytes in the data array
// Output:
//   - 16-bit CRC checksum (inverted at the end)
// ---------------------------------------------------------
uint16_t compute_crc_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;  // Step 1: Initialize CRC to all 1s (0xFFFF)

    // Step 2: Process each byte of the input data
    for (int i = 0; i < len; i++)
    {
        // Step 2.1: XOR the current byte into the top (high) 8 bits of CRC
        crc ^= ((uint16_t)data[i] << 8);

        // Step 2.2: Process each bit in the byte (8 times)
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                // Step 2.2.1: If the leftmost (highest) bit is 1
                // - Shift left by 1 bit
                // - XOR with the polynomial 0x1021
                crc = (crc << 1) ^ 0x1021;
            }

            else
            {
                // Step 2.2.2: If the leftmost bit is 0
                // - Only shift left by 1 bit (no XOR)
                crc <<= 1;
            }
        }
    }

    // Step 3: Invert all bits (bitwise NOT) and return
    return ~crc;
}

// ---------------------------------------------------------
// Function: COMMS_SendAX25
// Purpose:  Assemble and send an AX.25 frame over UART
// Inputs:
//   - huart: UART handle for transmission
//   - dest: Destination callsign (string)
//   - src: Source callsign (string)
//   - payload: Pointer to payload (Info field)
//   - len: Length of the payload
// ---------------------------------------------------------
void COMMS_SendAX25(UART_HandleTypeDef *huart, const char *dest, const char *src, const uint8_t *payload, uint8_t len) {
    uint8_t frame[MAX_FRAME_SIZE];  // Buffer to hold the entire AX.25 frame
    uint16_t i = 0;                 // Index pointer for building the frame

    // Step 1: Add the Start Flag (0x7E)
    frame[i++] = AX25_FLAG; // AX.25 requires a starting flag (01111110)

    // Step 2: Add Destination Address (6 characters + 1 SSID byte)
    for (int j = 0; j < 6; j++)
        frame[i++] = (j < strlen(dest)) ? (dest[j] << 1) : (' ' << 1);
        // Shift each character left by 1 bit according to AX.25 format
    frame[i++] = (0 << 1) | 0x00; // SSID byte: SSID = 0, end bit = 0 (more addresses follow)

    // Step 3: Add Source Address (6 characters + 1 SSID byte)
    for (int j = 0; j < 6; j++)
        frame[i++] = (j < strlen(src)) ? (src[j] << 1) : (' ' << 1);
        // Same left-shifting for source address
    frame[i++] = (0 << 1) | 0x01; // SSID byte: SSID = 0, end bit = 1 (this is the last address)

    // Step 4: Add Control Field
    frame[i++] = AX25_CONTROL; // Control field (0x03 for UI-frame)

    // Step 5: Add Protocol ID (PID Field)
    frame[i++] = AX25_PID;     // PID field (0xF0 means no Layer 3 protocol)

    // Step 6: Add Payload (Info field)
    for (int j = 0; j < len; j++)
        frame[i++] = payload[j]; // Copy each byte of payload into the frame

    // Step 7: Calculate and append CRC (FCS field)
    uint16_t crc = compute_crc_ccitt(&frame[1], i - 1);
    frame[i++] = crc & 0xFF;        // Append low byte first (little-endian)
    frame[i++] = (crc >> 8) & 0xFF; // Append high byte second

    // Step 8: Add the End Flag (0x7E)
    frame[i++] = AX25_FLAG; // Ending flag to mark frame boundary

    // Step 9: Send the entire frame over UART
    HAL_UART_Transmit(huart, frame, i, HAL_MAX_DELAY);
    // Transmit the frame buffer, total length = i
}


// ---------------------------------------------------------
// Function: COMMS_ReceiveAX25
// Purpose:  Receive and parse an AX.25 frame from UART
// Inputs:
//   - huart: UART handle for reception
//   - dest: Buffer to store destination callsign
//   - src: Buffer to store source callsign
//   - payload: Buffer to store received payload
//   - max_payload_len: Maximum allowed payload size
// Output:
//   - int: Number of payload bytes received, or -1 if error
// ---------------------------------------------------------
int COMMS_ReceiveAX25(UART_HandleTypeDef *huart, char *dest, char *src, uint8_t *payload, uint16_t max_payload_len) {
    uint8_t frame[MAX_FRAME_SIZE];
    uint16_t i = 0;
    uint8_t byte;

    // Step 1: Wait for start flag (0x7E)
    do {
        if (HAL_UART_Receive(huart, &byte, 1, HAL_MAX_DELAY) != HAL_OK)
            return -1;
    } while (byte != AX25_FLAG);

    // Step 2: Receive bytes until end flag (0x7E)
    while (1) {
        if (HAL_UART_Receive(huart, &byte, 1, HAL_MAX_DELAY) != HAL_OK)
            return -1;
        if (byte == AX25_FLAG)
            break; // End of frame
        if (i >= MAX_FRAME_SIZE)
            return -1; // Frame too large
        frame[i++] = byte;
    }

    // Step 3: Check frame length (minimum size check)
    if (i < (6+1)+(6+1)+2+2) // Dest+Src+Ctrl+PID+CRC
        return -1;

    uint16_t index = 0;

    // Step 4: Extract Destination Address
    for (int j = 0; j < 6; j++)
        dest[j] = (frame[index++] >> 1);
    dest[6] = '\0';
    index++; // Skip SSID byte

    // Step 5: Extract Source Address
    for (int j = 0; j < 6; j++)
        src[j] = (frame[index++] >> 1);
    src[6] = '\0';
    index++; // Skip SSID byte

    // Step 6: Verify Control Field and PID
    if (frame[index++] != AX25_CONTROL)
        return -1;
    if (frame[index++] != AX25_PID)
        return -1;

    // Step 7: Extract Payload
    uint16_t payload_len = i - index - 2; // Exclude CRC bytes
    if (payload_len > max_payload_len)
        return -1; // Payload too big

    for (int j = 0; j < payload_len; j++)
        payload[j] = frame[index++];

    // Step 8: Verify CRC
    uint16_t received_crc = frame[index] | (frame[index + 1] << 8);
    uint16_t computed_crc = compute_crc_ccitt(frame, i - 2);
    if (received_crc != computed_crc)
        return -1; // CRC mismatch

    return payload_len;
}
