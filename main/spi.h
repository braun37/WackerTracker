#ifndef __SPI_H__
#define __SPI_H__

void spi_send_cmd(spi_device_handle_t spi, const uint8_t cmd);
uint16_t spi_read_trans(spi_device_handle_t spi, const uint8_t addr, const int len);
void spi_write_trans(spi_device_handle_t spi, void *data, const int len);

#endif
