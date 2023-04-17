#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_nimble_hci.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"

#include "spi.h"

void spi_send_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
  spi_transaction_t t;
  esp_err_t ret;

  memset(&t, 0, sizeof(t));
  t.length = 8;
  t.tx_buffer = &cmd;
  t.flags = SPI_TRANS_CS_KEEP_ACTIVE;

  ret = spi_device_polling_transmit(spi, &t);
  ESP_ERROR_CHECK(ret);
}

uint16_t spi_read_trans(spi_device_handle_t spi, const uint8_t addr, const int len)
{
  // Acquire the bus for the device
  spi_device_acquire_bus(spi, portMAX_DELAY);

  // Send read command to the correct address
  spi_send_cmd(spi, (uint8_t)(0x80 | addr));

  spi_transaction_t t;
  esp_err_t ret;

  memset(&t, 0, sizeof(t));
  t.length = len;
  t.flags = SPI_TRANS_USE_RXDATA;

  ret=spi_device_polling_transmit(spi, &t);
  ESP_ERROR_CHECK(ret);

  // Release the bus
  spi_device_release_bus(spi);

  return *(uint16_t*)t.rx_data;
}

void spi_write_trans(spi_device_handle_t spi, void *data, const int len)
{
  // Acquire the bus for the device
  spi_device_acquire_bus(spi, portMAX_DELAY);

  spi_transaction_t t;
  esp_err_t ret;

  memset(&t, 0, sizeof(t));
  t.length = len;
  t.tx_buffer = data;
  t.flags = SPI_TRANS_USE_RXDATA;

  ret=spi_device_polling_transmit(spi, &t);
  ESP_ERROR_CHECK(ret);

  // Release the bus
  spi_device_release_bus(spi);
}
