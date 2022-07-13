


#ifndef INC_SHT31_STM32_H_
#define INC_SHT31_STM32_H_

#pragma once

#include <stdbool.h>

/********RTOS************/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "stm32f4xx_hal.h"

#ifndef SHT3X_I2C_TIMEOUT
#define SHT3X_I2C_TIMEOUT 0xffff
#endif

#define SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW 0x44
#define SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_HIGH 0x45

/**
 * Structure defining a handle describing a SHT3x device.
 */
typedef struct {

	/**
	 * The handle to the I2C bus for the device.
	 */
	I2C_HandleTypeDef *i2c_handle;

	/**
	 * The I2C device address.
	 * @see{PCA9865_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW} and @see{SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_HIGH}
	 */
	uint16_t device_address;

} sht3x_handle_t;

/**
 * Checks if an SHT3x is reachable using the given handle.
 * @param handle Handle to the SHT3x device.
 * @return True on success, false otherwise.
 */
bool sht3x_init(sht3x_handle_t *handle);

/**
 * Takes a single temperature and humidity measurement.
 * @param handle Handle to the SHT3x device.
 * @param temperature Pointer to the storage location for the sampled temperature.
 * @param humidity Pointer to the storage location for the sampled humidity.
 * @return True on success, false otherwise.
 */
bool sht3x_read_temperature_and_humidity(sht3x_handle_t *handle, float *temperature, float *humidity);

/**
 * Turns the SHT3x's internal heater on or of.
 * @param handle Handle to the SHT3x device.
 * @param enable True to enable to heater, false to disable it.
 * @return True on success, false otherwise.
 */
bool sht3x_set_header_enable(sht3x_handle_t *handle, bool enable);

/************RTOS SideProject**************/
/**
 * Use timer to get microsecond delay.
 * @param time 1 for 1 microsecond delay.
 */
void SHT31_Delay (uint16_t time);
#endif
