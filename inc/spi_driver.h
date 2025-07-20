/**
 * @file spi_driver.h
 * @brief SPI communication driver for BMI088
 * @author Yusuf Karaböcek
 * @date July 2025
 */

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <stdint.h>

/**
 * @brief SPI read wrapper function
 * @param dev_addr Device address
 * @param reg Register address
 * @param data Data buffer
 * @param len Data length
 * @return 0 on success, negative value on error
 */
int8_t spi_read_wrapper(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief SPI write wrapper function
 * @param dev_addr Device address
 * @param reg Register address
 * @param data Data buffer
 * @param len Data length
 * @return 0 on success, negative value on error
 */
int8_t spi_write_wrapper(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint16_t len);

/**
 * @brief Enable chip select (CS)
 */
void spi_cs_enable(void);

/**
 * @brief Disable chip select (CS)
 */
void spi_cs_disable(void);

#endif // SPI_DRIVER_H