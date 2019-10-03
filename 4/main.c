/**
 * Copyright (c) 2009 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
*
*/

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"
 #include "nrf_delay.h"
#include "nrfx_spim.h"
#include "nrf_drv_spi.h"
#include "bmi160.h"

#define SPI_INSTANCE 0 // SPI instance index. We use SPI master 0
#define SPI_SS_PIN 26
#define SPI_MISO_PIN 23
#define SPI_MOSI_PIN 24
#define SPI_SCK_PIN 22
//SPI instance
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
//Flag used to indicate that SPI instance completed the transfer
static volatile bool spi_xfer_done;

static uint8_t SPI_RX_Buffer[100]; // Allocate a buffer for SPI reads
struct bmi160_dev sensor; // An instance of bmi160 sensor


/**
 * SPI user event handler.
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	spi_xfer_done = true; // Set a flag when transfer is done
}

/**
 * Function for setting up the SPI communication.
 */
uint32_t spi_config()
{
	uint32_t err_code;
	// Use nRF's default configurations
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	// Define each GPIO pin
	spi_config.ss_pin = SPI_SS_PIN;
	spi_config.miso_pin = SPI_MISO_PIN;
	spi_config.mosi_pin = SPI_MOSI_PIN;
	spi_config.sck_pin = SPI_SCK_PIN;
	// Initialize the SPI peripheral and give it a function pointer to
	// it’s event handler
	err_code = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);
	return err_code;
}


/**
 * Function for writing to the BMI160 via SPI.
 */
int8_t bmi160_spi_bus_write(uint8_t hw_addr, uint8_t reg_addr, uint8_t *reg_data,
uint16_t cnt)
{
	spi_xfer_done = false; // set the flag down during transfer
	int32_t error = 0;
	// Allocate array, which lenght is address + number of data bytes to be sent
	uint8_t tx_buff[cnt+1];
	uint16_t stringpos;
	// AND address with 0111 1111; set msb to '0' (write operation)
	 tx_buff[0] = reg_addr & 0x7F;
	for (stringpos = 0; stringpos < cnt; stringpos++) {
	 tx_buff[stringpos+1] = *(reg_data + stringpos);
	}
	// Do the actual SPI transfer
	 nrf_drv_spi_transfer(&spi, tx_buff, cnt+1, NULL, 0);
	while (!spi_xfer_done) {}; // Loop until the transfer is complete
	return (int8_t)error;
}

/**
 * Function for reading from the BMI160 via SPI.
 */
int8_t bmi160_spi_bus_read(uint8_t hw_addr, uint8_t reg_addr, uint8_t *reg_data,
uint16_t len)
{
	 spi_xfer_done = false; // set the flag down during transfer
	int32_t error = 0;
	uint8_t tx_buff = reg_addr | 0x80; // OR address with 1000 0000; Read -> set msb to '1';
	uint8_t * rx_buff_pointer;
	uint16_t stringpos;
	 rx_buff_pointer = (uint8_t *) (SPI_RX_Buffer);
	// Do the actual SPI transfer
	 nrf_drv_spi_transfer(&spi, &tx_buff, 1, rx_buff_pointer, len+1);
	while (!spi_xfer_done) {} // Loop until the transfer is complete
	// Copy received bytes to reg_data
	for (stringpos = 0; stringpos < len; stringpos++)
	*(reg_data + stringpos) = SPI_RX_Buffer[stringpos + 1];
	return (int8_t)error;
}

/**
 * Function for configuring the sensor
 */
int8_t sensor_config()
{
	int8_t rslt = BMI160_OK;
	sensor.id = 0; // We use SPI so id == 0
	sensor.interface = BMI160_SPI_INTF;
	// Give the driver the correct interfacing functions
	sensor.read = bmi160_spi_bus_read;
	sensor.write = bmi160_spi_bus_write;
	sensor.delay_ms = nrf_delay_ms;
	// Initialize the sensor and check if everything went ok
	rslt = bmi160_init(&sensor);
	return rslt;
}



/**
 * @brief Function for application main entry.
 */
int main(void)
{
	spi_config();
	sensor_config();

	while (true)
	{
			// Do nothing.
	}
}
/** @} */
