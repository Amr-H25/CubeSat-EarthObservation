/**
 * @file simple_gps.c
 * @brief Simple driver implementation for GPS module
 * @author CubeSat Project Team
 */

#include "gps.h"
#include "math.h"

/**
 * @brief Initialize GPS driver
 */
GPS_Status GPS_Init(GPS_Handle* gps, UART_HandleTypeDef* huart) {
    if (gps == NULL || huart == NULL) {
        return GPS_ERROR;
    }

    // Initialize GPS handle
    memset(gps, 0, sizeof(GPS_Handle));
    gps->huart = huart;
    gps->buffer_head = 0;
    gps->buffer_tail = 0;
    gps->nmea_index = 0;
    gps->fix_available = 0;

    // Start UART reception in interrupt mode (single byte)
    if (HAL_UART_Receive_IT(gps->huart, &gps->rx_byte, 1) != HAL_OK) {
        return GPS_ERROR;
    }

    return GPS_OK;
}

/**
 * @brief Process incoming GPS data
 */
GPS_Status GPS_Process(GPS_Handle* gps) {
    // Process data in circular buffer
    while (gps->buffer_head != gps->buffer_tail) {
        uint8_t byte = gps->buffer[gps->buffer_tail];
        gps->buffer_tail = (gps->buffer_tail + 1) % GPS_BUFFER_SIZE;

        // NMEA sentence starts with '$'
        if (byte == GPS_NMEA_START) {
            gps->nmea_index = 0;
            gps->nmea_buffer[gps->nmea_index++] = byte;
        }
        // Add byte to NMEA buffer if we've seen a start character
        else if (gps->nmea_index > 0) {
            // Check for buffer overflow
            if (gps->nmea_index >= GPS_NMEA_SIZE - 1) {
                gps->nmea_index = 0; // Reset and wait for next sentence
                continue;
            }

            gps->nmea_buffer[gps->nmea_index++] = byte;

            // End of NMEA sentence (newline or carriage return)
            if (byte == GPS_NMEA_END || byte == '\r') {
                gps->nmea_buffer[gps->nmea_index] = '\0'; // Null terminate

                // Parse the NMEA sentence
                NMEA_Type type = GPS_ParseNMEASentence(gps, gps->nmea_buffer, NULL);

                if (type != NMEA_UNKNOWN) {
                    // Update last update timestamp
                    gps->last_update = HAL_GetTick();
                }

                // Reset NMEA buffer index
                gps->nmea_index = 0;
            }
        }
    }

    return GPS_OK;
}

/**
 * @brief UART RX complete callback - to be called from HAL_UART_RxCpltCallback
 */
void GPS_UART_RxCpltCallback(GPS_Handle* gps) {
    // Add received byte to circular buffer
    gps->buffer[gps->buffer_head] = gps->rx_byte;
    gps->buffer_head = (gps->buffer_head + 1) % GPS_BUFFER_SIZE;

    // Restart UART reception
    HAL_UART_Receive_IT(gps->huart, &gps->rx_byte, 1);
}

/**
 * @brief Parse NMEA sentence and direct to appropriate parser
 */
NMEA_Type GPS_ParseNMEASentence(GPS_Handle* gps, char* nmea, UART_HandleTypeDef* debug_huart) {
    // Check if the sentence starts with "$GP"
    if (strncmp(nmea, "$GP", 3) != 0) {
        return NMEA_UNKNOWN;
    }

    // Identify NMEA sentence type
    if (strncmp(nmea + 3, "GGA", 3) == 0) {
        GPS_ParseGPGGA(gps, nmea, debug_huart);
        return NMEA_GPGGA;
    } else if (strncmp(nmea + 3, "RMC", 3) == 0) {
        GPS_ParseGPRMC(gps, nmea);
        return NMEA_GPRMC;
    }

    return NMEA_UNKNOWN;
}

/**
 * @brief Parse GPGGA sentence (Global Positioning System Fix Data)
 * Format: $GPGGA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,geoid,M,age,ref*cs
 * Example: $GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,000062
 */
GPS_Status GPS_ParseGPGGA(GPS_Handle* gps, char* nmea, UART_HandleTypeDef* debug_huart) {
    char* token;
    char* saveptr;
    int token_index = 0;
    char nmea_copy[GPS_NMEA_SIZE]; // Create a copy of the NMEA string to avoid modifying the original
    char debug_buffer[128]; // Local buffer for debug messages

    // Copy the NMEA string to avoid strtok_r modifying the original
    strncpy(nmea_copy, nmea, GPS_NMEA_SIZE - 1);
    nmea_copy[GPS_NMEA_SIZE - 1] = '\0'; // Ensure null termination

    // First token is the message ID ($GPGGA)
    token = strtok_r(nmea_copy, ",", &saveptr);
    if (token == NULL) return GPS_INVALID_DATA;

    // Parse remaining tokens
    while ((token = strtok_r(NULL, ",*", &saveptr)) != NULL) {
        token_index++;

        switch (token_index) {
            case 1: // Time (format: hhmmss.sss)
                if (strlen(token) >= 6) {
                    gps->time.hour = (token[0] - '0') * 10 + (token[1] - '0');
                    gps->time.minute = (token[2] - '0') * 10 + (token[3] - '0');
                    gps->time.second = (token[4] - '0') * 10 + (token[5] - '0');

                    if (strlen(token) > 7) { // Check if milliseconds present
                        gps->time.millisecond = atoi(&token[7]);
                    }
                }
                break;

            case 2: // Latitude (format: ddmm.mmmm)
                if (strlen(token) > 0) {
                    // Extract degrees (first 2 digits)
                    int degrees = ((token[0] - '0') * 10) + (token[1] - '0');

                    // Extract minutes (remaining digits)
                    double minutes = ((token[2] - '0') * 10) + (token[3] - '0');  // whole minutes
                    if (token[4] == '.') {  // handle decimal part
                        minutes += (token[5] - '0') * 0.1000;
                        minutes += (token[6] - '0') * 0.0100;
                        minutes += (token[7] - '0') * 0.0010;
                        minutes += (token[8] - '0') * 0.0001;
                    }

                    // Convert to decimal degrees
                    gps->position.latitude = degrees + (minutes / 60.0);

                    // Debug output if debug UART is provided
                    if (debug_huart != NULL) {
                        sprintf(debug_buffer, "Debug Lat: str=%s, deg=%d, min=%.4f, final=%.6f\r\n",
                                token, degrees, minutes, gps->position.latitude);
                        HAL_UART_Transmit(debug_huart, (uint8_t*)debug_buffer, strlen(debug_buffer), 100);
                    }
                }
                break;

            case 3: // N/S indicator
                if (token[0] == 'S') {
                    gps->position.latitude = -gps->position.latitude;
                }
                break;

            case 4: // Longitude (format: dddmm.mmmm)
                if (strlen(token) > 0) {
                    // Extract degrees (first 3 digits)
                    int degrees = ((token[0] - '0') * 100) +
                                 ((token[1] - '0') * 10) +
                                 (token[2] - '0');

                    // Extract minutes (remaining digits)
                    double minutes = ((token[3] - '0') * 10) + (token[4] - '0');  // whole minutes
                    if (token[5] == '.') {  // handle decimal part
                        minutes += (token[6] - '0') * 0.1000;
                        minutes += (token[7] - '0') * 0.0100;
                        minutes += (token[8] - '0') * 0.0010;
                        minutes += (token[9] - '0') * 0.0001;
                    }

                    // Convert to decimal degrees
                    gps->position.longitude = degrees + (minutes / 60.0);

                    // Debug output if debug UART is provided
                    if (debug_huart != NULL) {
                        sprintf(debug_buffer, "Debug Lon: str=%s, deg=%d, min=%.4f, final=%.6f\r\n",
                                token, degrees, minutes, gps->position.longitude);
                        HAL_UART_Transmit(debug_huart, (uint8_t*)debug_buffer, strlen(debug_buffer), 100);
                    }
                }
                break;

            case 5: // E/W indicator
                if (token[0] == 'W') {
                    gps->position.longitude = -gps->position.longitude;
                }
                break;

            case 6: // Fix quality
                gps->position.fix = (GPS_FixType)atoi(token);
                gps->fix_available = (gps->position.fix > GPS_FIX_NONE) ? 1 : 0;
                gps->position.valid = gps->fix_available;
                break;

            case 7: // Number of satellites
                gps->position.satellites = atoi(token);
                break;

            case 8: // HDOP
                gps->position.hdop = atof(token);
                break;

            case 9: // Altitude
                gps->position.altitude = atof(token);
                break;
        }
    }

    return GPS_OK;
}

/**
 * @brief Parse GPRMC sentence (Recommended Minimum Navigation Information)
 * Format: $GPRMC,time,status,lat,N/S,lon,E/W,spd,cog,date,mv,mvE/W,mode*cs
 * Example: $GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C
 */
GPS_Status GPS_ParseGPRMC(GPS_Handle* gps, char* nmea) {
    char* token;
    char* saveptr;
    int token_index = 0;
    char nmea_copy[GPS_NMEA_SIZE]; // Create a copy of the NMEA string to avoid modifying the original

    // Copy the NMEA string to avoid strtok_r modifying the original
    strncpy(nmea_copy, nmea, GPS_NMEA_SIZE - 1);
    nmea_copy[GPS_NMEA_SIZE - 1] = '\0'; // Ensure null termination

    // First token is the message ID ($GPRMC)
    token = strtok_r(nmea_copy, ",", &saveptr);
    if (token == NULL) return GPS_INVALID_DATA;

    // Parse remaining tokens
    while ((token = strtok_r(NULL, ",*", &saveptr)) != NULL) {
        token_index++;

        switch (token_index) {
            case 1: // Time (format: hhmmss.sss)
                if (strlen(token) >= 6) {
                    gps->time.hour = (token[0] - '0') * 10 + (token[1] - '0');
                    gps->time.minute = (token[2] - '0') * 10 + (token[3] - '0');
                    gps->time.second = (token[4] - '0') * 10 + (token[5] - '0');

                    if (strlen(token) > 7) { // Check if milliseconds present
                        gps->time.millisecond = atoi(&token[7]);
                    }
                }
                break;

            case 2: // Status (A=valid, V=invalid)
                gps->position.valid = (token[0] == 'A') ? 1 : 0;
                break;

            case 3: // Latitude (format: ddmm.mmmm)
                if (strlen(token) > 0) {
                    char deg_str[3];
                    strncpy(deg_str, token, 2);
                    deg_str[2] = '\0';

                    int degrees = atoi(deg_str);
                    double minutes = atof(token + 2);  // Get the minutes part
                    gps->position.latitude = degrees + (minutes / 60.0);
                }
                break;

            case 4: // N/S indicator
                if (token[0] == 'S') {
                    gps->position.latitude = -gps->position.latitude;
                }
                break;

            case 5: // Longitude (format: dddmm.mmmm)
                if (strlen(token) > 0) {
                    char deg_str[4];
                    strncpy(deg_str, token, 3);
                    deg_str[3] = '\0';

                    int degrees = atoi(deg_str);
                    double minutes = atof(token + 3);  // Get the minutes part
                    gps->position.longitude = degrees + (minutes / 60.0);
                }
                break;

            case 6: // E/W indicator
                if (token[0] == 'W') {
                    gps->position.longitude = -gps->position.longitude;
                }
                break;

            case 7: // Speed over ground (knots)
                gps->position.speed = atof(token);
                break;

            case 8: // Course over ground (degrees)
                gps->position.course = atof(token);
                break;

            case 9: // Date (format: ddmmyy)
                if (strlen(token) == 6) {
                    gps->time.day = (token[0] - '0') * 10 + (token[1] - '0');
                    gps->time.month = (token[2] - '0') * 10 + (token[3] - '0');
                    // Convert 2-digit year to full year
                    int year = (token[4] - '0') * 10 + (token[5] - '0');
                    gps->time.year = 2000 + year; // Assuming years after 2000
                }
                break;
        }
    }

    // If position is valid, set fix available
    if (gps->position.valid) {
        gps->fix_available = 1;
    }

    return GPS_OK;
}

/**
 * @brief Get current GPS position
 */
GPS_Status GPS_GetPosition(GPS_Handle* gps, GPS_Position* position) {
    if (!gps->fix_available) {
        return GPS_NO_FIX;
    }

    // Copy position data
    memcpy(position, &gps->position, sizeof(GPS_Position));

    return GPS_OK;
}

/**
 * @brief Get current GPS time
 */
GPS_Status GPS_GetTime(GPS_Handle* gps, GPS_Time* time) {
    // Copy time data
    memcpy(time, &gps->time, sizeof(GPS_Time));

    return GPS_OK;
}

/**
 * @brief Check if GPS has a valid fix
 */
uint8_t GPS_HasFix(GPS_Handle* gps) {
    return gps->fix_available;
}

/**
 * @brief Debug function to print parsed GPS data via UART
 */
void GPS_DebugPrint(GPS_Handle* gps, UART_HandleTypeDef* debug_huart) {
    char buffer[256];

    if (debug_huart == NULL) {
        return;
    }

    // Position info
    sprintf(buffer, "GPS Position:\r\n");
    HAL_UART_Transmit(debug_huart, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "  Fix: %s\r\n", gps->fix_available ? "Valid" : "Invalid");
    HAL_UART_Transmit(debug_huart, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "  Latitude: %.6f\r\n", gps->position.latitude);
    HAL_UART_Transmit(debug_huart, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "  Longitude: %.6f\r\n", gps->position.longitude);
    HAL_UART_Transmit(debug_huart, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "  Altitude: %.2f m\r\n", gps->position.altitude);
    HAL_UART_Transmit(debug_huart, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "  Satellites: %d\r\n", gps->position.satellites);
    HAL_UART_Transmit(debug_huart, (uint8_t*)buffer, strlen(buffer), 100);

    // Time info
    sprintf(buffer, "GPS Time:\r\n");
    HAL_UART_Transmit(debug_huart, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "  %02d/%02d/%04d %02d:%02d:%02d.%03d\r\n",
            gps->time.day, gps->time.month, gps->time.year,
            gps->time.hour, gps->time.minute, gps->time.second, gps->time.millisecond);
    HAL_UART_Transmit(debug_huart, (uint8_t*)buffer, strlen(buffer), 100);
}

/**
 * @brief Convert GPS coordinates to ECEF coordinates
 */
GPS_Status GPS_GetECEFPosition(GPS_Handle* gps, ECEF_Position* ecef) {
    if (!gps->fix_available) {
        return GPS_NO_FIX;
    }

    // Convert latitude and longitude to radians
    double lat = gps->position.latitude * PI / 180.0;
    double lon = gps->position.longitude * PI / 180.0;
    double alt = gps->position.altitude;

    // Calculate N (radius of curvature in prime vertical)
    double N = GPS_WGS84_A / sqrt(1 - GPS_WGS84_E2 * sin(lat) * sin(lat));

    // Calculate ECEF coordinates
    ecef->x = (N + alt) * cos(lat) * cos(lon);
    ecef->y = (N + alt) * cos(lat) * sin(lon);
    ecef->z = (N * (1 - GPS_WGS84_E2) + alt) * sin(lat);

    return GPS_OK;
}

/**
 * @brief Convert GPS coordinates to ECI coordinates
 * @note This is a simplified conversion that doesn't account for precession,
 *       nutation, or polar motion. For more accurate results, these effects
 *       should be included.
 */
GPS_Status GPS_GetECIPosition(GPS_Handle* gps, ECI_Position* eci) {
    if (!gps->fix_available) {
        return GPS_NO_FIX;
    }

    // First get ECEF coordinates
    ECEF_Position ecef;
    GPS_Status status = GPS_GetECEFPosition(gps, &ecef);
    if (status != GPS_OK) {
        return status;
    }

    // Calculate GMST (Greenwich Mean Sidereal Time)
    // This is a simplified calculation - for better accuracy, use a proper GMST calculation
    double hours = gps->time.hour + gps->time.minute/60.0 + gps->time.second/3600.0;
    double days = gps->time.day + (hours/24.0);
    double months = gps->time.month;
    double years = gps->time.year;

    // Calculate Julian Date
    double jd = 367 * years - (int)((7 * (years + (int)((months + 9) / 12))) / 4) +
                (int)((275 * months) / 9) + days + 1721013.5;

    // Calculate GMST in radians
    double t = (jd - 2451545.0) / 36525.0;
    double gmst = 280.46061837 + 360.98564736629 * (jd - 2451545.0) +
                  0.000387933 * t * t - t * t * t / 38710000.0;

    // Normalize GMST to 0-360 degrees
    while (gmst > 360.0) gmst -= 360.0;
    while (gmst < 0.0) gmst += 360.0;

    // Convert to radians
    double theta = gmst * PI / 180.0;

    // Convert ECEF to ECI
    eci->x = ecef.x * cos(theta) - ecef.y * sin(theta);
    eci->y = ecef.x * sin(theta) + ecef.y * cos(theta);
    eci->z = ecef.z;

    // Calculate velocities (simplified - only accounts for Earth's rotation)
    double omega_earth = EARTH_ROTATION_RATE; // Earth's rotation rate in rad/s
    eci->vx = -omega_earth * eci->y;
    eci->vy = omega_earth * eci->x;
    eci->vz = 0.0;

    return GPS_OK;
}
