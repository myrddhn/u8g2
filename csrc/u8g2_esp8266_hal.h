/*
 * u8g2_esp8266_hal.h
 *
 *  Created on: Feb 12, 2017
 *      Author: kolban
 */

#ifndef U8G2_ESP8266_HAL_H_
#define U8G2_ESP8266_HAL_H_
#include "u8g2.h"

#include "driver/gpio.h"
/* Not implemented
#include "driver/spi_master.h"
*/
#include "driver/i2c.h"

#define SDA_PIN GPIO_NUM_5
#define SCL_PIN GPIO_NUM_4

#define tag "SH1106"

#define GPIO_OUTPUT_PIN_SEL  ((1<<SDA_PIN) | (1<<SCL_PIN))
#define U8G2_ESP8266_HAL_UNDEFINED (-1)

#define I2C_MASTER_NUM I2C_NUM_0           //  I2C port number for master dev
#define ACK_CHECK_EN   0x1                 //  I2C master will check ack from slave
#define ACK_CHECK_DIS  0x0                 //  I2C master will not check ack from slave

typedef struct {
	gpio_num_t clk;
	gpio_num_t mosi;
	gpio_num_t sda; // data for I²C
	gpio_num_t scl; // clock for I²C
	gpio_num_t cs;
	gpio_num_t reset;
	gpio_num_t dc;
} u8g2_esp8266_hal_t ;

#define U8G2_ESP8266_HAL_DEFAULT {U8G2_ESP8266_HAL_UNDEFINED, U8G2_ESP8266_HAL_UNDEFINED, SDA_PIN, SCL_PIN, U8G2_ESP8266_HAL_UNDEFINED, U8G2_ESP8266_HAL_UNDEFINED, U8G2_ESP8266_HAL_UNDEFINED }

void u8g2_esp8266_hal_init(u8g2_esp8266_hal_t u8g2_esp8266_hal_param);
//uint8_t u8g2_esp8266_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_esp8266_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_esp8266_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
#endif /* U8G2_ESP8266_HAL_H_ */
