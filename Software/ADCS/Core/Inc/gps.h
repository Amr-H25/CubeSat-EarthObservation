/**
 * @file simple_gps.h
 * @brief Simple driver for GPS module
 * @author CubeSat Project Team
 */

#ifndef SIMPLE_GPS_H_
#define SIMPLE_GPS_H_

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* GPS buffer size definitions */
#define GPS_BUFFER_SIZE     512     // Buffer for UART reception
#define GPS_NMEA_SIZE       128     // Maximum size of a NMEA sentence
#define GPS_NMEA_START      '$'     // NMEA sentence start character
#define GPS_NMEA_END        '\n'    // NMEA sentence end character

#define GPS_WGS84_A     6378137.0       // Semi-major axis [m]
#define GPS_WGS84_B     6356752.314245  // Semi-minor axis [m]
#define GPS_WGS84_E     0.081819190842622    // Eccentricity
#define GPS_WGS84_E2    0.006694379990141    // Eccentricity squared
#define PI              3.14159265358979323846
#define EARTH_ROTATION_RATE 7.2921150e-5      // Earth rotation rate [rad/s]

/* Status enum for GPS operations */
typedef enum {
    GPS_OK = 0,
    GPS_ERROR,
    GPS_TIMEOUT,
    GPS_NO_FIX,
    GPS_INVALID_DATA
} GPS_Status;

/* Types of fix */
typedef enum {
    GPS_FIX_NONE = 0,
    GPS_FIX_GPS = 1,
    GPS_FIX_DGPS = 2
} GPS_FixType;

/* NMEA sentence types of interest */
typedef enum {
    NMEA_UNKNOWN = 0,
    NMEA_GPGGA,
    NMEA_GPRMC
} NMEA_Type;

/* Position structure */
typedef struct {
    double latitude;      // Latitude in degrees (positive = North, negative = South)
    double longitude;     // Longitude in degrees (positive = East, negative = West)
    double altitude;      // Altitude in meters above mean sea level
    double speed;         // Speed over ground in knots
    double course;        // Course over ground in degrees
    uint8_t satellites;   // Number of satellites used in the solution
    double hdop;          // Horizontal dilution of precision
    GPS_FixType fix;      // Type of GPS fix
    uint8_t valid;        // Is the position valid?
} GPS_Position;

/* Time structure */
typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
    uint8_t day;
    uint8_t month;
    uint16_t year;    // Full year (e.g., 2025)
} GPS_Time;

/* GPS driver handle structure */
typedef struct {
    UART_HandleTypeDef* huart;       // UART handle for GPS
    uint8_t buffer[GPS_BUFFER_SIZE]; // Circular buffer for UART reception
    uint16_t buffer_head;            // Head index of circular buffer
    uint16_t buffer_tail;            // Tail index of circular buffer
    uint8_t rx_byte;                 // Single byte for UART receive
    char nmea_buffer[GPS_NMEA_SIZE]; // Buffer for current NMEA sentence
    uint16_t nmea_index;             // Current index in NMEA buffer
    GPS_Position position;           // Current position
    GPS_Time time;                   // Current GPS time
    uint8_t fix_available;           // Whether we have a valid fix
    uint32_t last_update;            // Timestamp of last position update
} GPS_Handle;

typedef struct {
    double x;    // [m]
    double y;    // [m]
    double z;    // [m]
} ECEF_Position;

typedef struct {
    double x;    // [m]
    double y;    // [m]
    double z;    // [m]
    double vx;   // [m/s]
    double vy;   // [m/s]
    double vz;   // [m/s]
} ECI_Position;

/* Function prototypes */

/**
 * @brief Initialize GPS driver
 * @param gps Pointer to GPS handle structure
 * @param huart Pointer to UART handle for GPS communication
 * @return GPS_OK if successful, GPS_ERROR otherwise
 */
GPS_Status GPS_Init(GPS_Handle* gps, UART_HandleTypeDef* huart);

/**
 * @brief Process GPS data - should be called regularly in main loop
 * @param gps Pointer to GPS handle structure
 * @return GPS_OK if successful, otherwise error code
 */
GPS_Status GPS_Process(GPS_Handle* gps);

/**
 * @brief Get current GPS position
 * @param gps Pointer to GPS handle structure
 * @param position Pointer to position structure to be filled
 * @return GPS_OK if valid position available, GPS_NO_FIX otherwise
 */
GPS_Status GPS_GetPosition(GPS_Handle* gps, GPS_Position* position);

/**
 * @brief Get current GPS time
 * @param gps Pointer to GPS handle structure
 * @param time Pointer to time structure to be filled
 * @return GPS_OK if valid time available, GPS_ERROR otherwise
 */
GPS_Status GPS_GetTime(GPS_Handle* gps, GPS_Time* time);

/**
 * @brief UART RX complete callback - to be called from HAL_UART_RxCpltCallback
 * @param gps Pointer to GPS handle structure
 */
void GPS_UART_RxCpltCallback(GPS_Handle* gps);

/**
 * @brief Check if GPS has a valid fix
 * @param gps Pointer to GPS handle structure
 * @return 1 if GPS has a valid fix, 0 otherwise
 */
uint8_t GPS_HasFix(GPS_Handle* gps);

/**
 * @brief Debug function to print parsed GPS data via UART
 * @param gps Pointer to GPS handle structure
 * @param debug_huart UART handle to output debug information
 */
void GPS_DebugPrint(GPS_Handle* gps, UART_HandleTypeDef* debug_huart);

/**
 * @brief Convert GPS coordinates to ECEF coordinates
 * @param gps Pointer to GPS handle structure
 * @param ecef Pointer to ECEF position structure to be filled
 * @return GPS_OK if successful, GPS_NO_FIX if no valid fix available
 */
GPS_Status GPS_GetECEFPosition(GPS_Handle* gps, ECEF_Position* ecef);

/**
 * @brief Convert GPS coordinates to ECI coordinates
 * @param gps Pointer to GPS handle structure
 * @param eci Pointer to ECI position structure to be filled
 * @return GPS_OK if successful, GPS_NO_FIX if no valid fix available
 */
GPS_Status GPS_GetECIPosition(GPS_Handle* gps, ECI_Position* eci);

/* Internal functions */

/**
 * @brief Parse NMEA sentence and direct to appropriate parser
 * @param gps Pointer to GPS handle structure
 * @param nmea NMEA sentence string to parse
 * @param debug_huart Optional UART handle for debug output (can be NULL)
 * @return Type of NMEA sentence parsed
 */
NMEA_Type GPS_ParseNMEASentence(GPS_Handle* gps, char* nmea, UART_HandleTypeDef* debug_huart);

/**
 * @brief Parse GPGGA sentence
 * @param gps Pointer to GPS handle structure
 * @param nmea NMEA sentence string to parse
 * @param debug_huart Optional UART handle for debug output (can be NULL)
 * @return GPS_OK if successful, otherwise error code
 */
GPS_Status GPS_ParseGPGGA(GPS_Handle* gps, char* nmea, UART_HandleTypeDef* debug_huart);

/**
 * @brief Parse GPRMC sentence
 * @param gps Pointer to GPS handle structure
 * @param nmea NMEA sentence string to parse
 * @return GPS_OK if successful
 * * @brief Parse GPRMC sentence
 * @param gps Pointer to GPS handle structure
 * @param nmea NMEA sentence string to parse
 * @return GPS_OK if successful, otherwise error code
 */
GPS_Status GPS_ParseGPRMC(GPS_Handle* gps, char* nmea);

#endif /* SIMPLE_GPS_H_ */
