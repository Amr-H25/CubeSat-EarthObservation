/*
 * config.h
 *
 *  Created on: Jul 4, 2025
 *      Author: Amr_H
 */


#ifndef CONFIG_H
#define CONFIG_H

// LM35 Configuration
#define LM35_ADC_PIN        GPIO_PIN_0
#define LM35_ADC_PORT       GPIOA
#define LM35_ADC_CHANNEL    ADC_CHANNEL_0

// EPS timing configuration
#define EPS_READ_INTERVAL_MS    1000  // Read sensors every 1 second
#define EPS_I2C_TIMEOUT_MS      100   // I2C timeout

// I2C Communication addresses (adjust as needed)
#define EPS_I2C_ADDRESS         0x20  // EPS address on I2C bus

// Power management thresholds
#define BATTERY_LOW_THRESHOLD   3.0f  // Volts
#define BATTERY_CRITICAL_THRESHOLD  2.5f  // Volts

#endif /* CONFIG_H */
