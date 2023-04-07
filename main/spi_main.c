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

#include "LSM6DSM.h"

#define BYTE 8
#define HALFWORD 16
#define WORD 32

#define HOST    HSPI_HOST

#define PIN_MISO 25
#define PIN_MOSI 23
#define PIN_CLK  19
#define PIN_CS0  22
#define PIN_CS1  21

#define PIN_SEN0_INT1 32
#define PIN_SEN1_INT1 33

#define PARALLEL_LINES 16

//******************************
// DRAM Setup
//******************************
#define DRAM_CNT_HALFWORD 4096

uint16_t *ram = NULL;

uint16_t ram_rd_idx = 0;
uint16_t ram_wr_idx = 0;

//******************************
// Sensor Setup
//******************************

// Instruction Struct
typedef struct _inst {
  uint8_t addr;
  uint8_t data;
} inst;

static inst setup[] = {
  // Sensor Setup
  {LSM6DSM_CTRL3_C, 0x45}, // Block Update=TRUE

  // GYRO Setup
  {LSM6DSM_CTRL2_G, 0x20}, // ODR=26 Hz, FS_G=250 dps

  // Accel Setup
  {LSM6DSM_CTRL1_XL, 0x2E}, // ODR=26 Hz, FS=+-8g, LPF1_BW_SEL=1

  // FIFO Setup
  {LSM6DSM_FIFO_CTRL2, 0x04}, // WATERMARK=50%
  {LSM6DSM_FIFO_CTRL3, 0x09}, // GYRO DEC=NONE, XL DEC=NONE
  {LSM6DSM_FIFO_CTRL5, 0x16}, // ODR=26 Hz, FIFO_MODE=CONTINOUS MODE
  
  // INT1 Setup
  {LSM6DSM_INT1_CTRL, 0x08} // INT1 on FTH
};

// Helper Functions
int _num_setup_inst() {
  return sizeof(setup) / sizeof(inst);
}

//******************************
// SPI Helper Functions
//******************************

void _spi_send_cmd(spi_device_handle_t spi, const uint8_t cmd)
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

uint32_t _spi_read_trans(spi_device_handle_t spi, const uint8_t addr, const int len)
{
  // Acquire the bus for the device
  spi_device_acquire_bus(spi, portMAX_DELAY);

  // Send read command to the correct address
  _spi_send_cmd(spi, (uint8_t)(0x80 | addr));

  spi_transaction_t t;
  esp_err_t ret;

  memset(&t, 0, sizeof(t));
  t.length = len;
  t.flags = SPI_TRANS_USE_RXDATA;

  ret=spi_device_polling_transmit(spi, &t);
  ESP_ERROR_CHECK(ret);

  // Release the bus
  spi_device_release_bus(spi);

  return *(uint32_t*)t.rx_data;
}

void _spi_write_trans(spi_device_handle_t spi, void *data, const int len)
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

//******************************
// Sensor Helper Functions
//******************************

void _setup_sensor(spi_device_handle_t spi) {
  int numInst = _num_setup_inst();

  for(int i=0; i<numInst; i++) {
    _spi_write_trans(spi, &setup[i], HALFWORD);
  }
}

void _fifo_read(spi_device_handle_t spi, uint8_t num) {
  uint16_t fifoEntries = _spi_read_trans(spi, LSM6DSM_FIFO_STATUS1, HALFWORD) & 0x7FFF;

  printf("FIFO ENTRIES = %d\n", fifoEntries);

  for(int i=0; i<(fifoEntries/6)*6; i++) {
    uint16_t data = _spi_read_trans(spi, LSM6DSM_FIFO_DATA_OUT_L, HALFWORD);
    ram[ram_wr_idx] = (data&~0x3) | (num&0x3);
    ram_wr_idx = (ram_wr_idx == DRAM_CNT_HALFWORD) ? 0 : ram_wr_idx+1;
    assert(ram_wr_idx != ram_rd_idx);
  }
}

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
  uint32_t gpio_pin = (uint32_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_pin, NULL);
}

spi_device_handle_t sens0;
spi_device_handle_t sens1;

static void gpio_task_example(void* arg) {
  uint32_t io_pin;
  for(;;) {
    if(xQueueReceive(gpio_evt_queue, &io_pin, portMAX_DELAY)) {
        printf("GPIO[%"PRIu32"] intr, val: %d\n", io_pin, gpio_get_level(io_pin));
        
        if(gpio_get_level(io_pin) == 0) continue;

        switch(io_pin) {
          case 32: _fifo_read(sens0, 0);
                   break;
          case 33: _fifo_read(sens1, 1);
                   break;
        }
    }
  }
}

void app_main(void)
{
  esp_err_t ret;

  {
    spi_bus_config_t buscfg = {
      .miso_io_num=PIN_MISO,
      .mosi_io_num=PIN_MOSI,
      .sclk_io_num=PIN_CLK,
      .quadwp_io_num=-1,
      .quadhd_io_num=-1,
      .max_transfer_sz=PARALLEL_LINES*320*2+8
    };

    //Initialize the SPI bus
    ret=spi_bus_initialize(HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
  }

  {
    spi_device_interface_config_t devcfg = {
      .clock_speed_hz=5*1000*1000,        // Clock out at 5 MHz
      .mode=0,                            // SPI mode 0
      .spics_io_num=PIN_CS0,              // CS pin
      .queue_size=7,                      // We want to be able to queue 7 transactions at a time
    };

    //Attach the SENS0 to the SPI bus
    ret=spi_bus_add_device(HOST, &devcfg, &sens0);
    ESP_ERROR_CHECK(ret);
  }

  {
    spi_device_interface_config_t devcfg = {
      .clock_speed_hz=5*1000*1000,        // Clock out at 5 MHz
      .mode=0,                            // SPI mode 0
      .spics_io_num=PIN_CS1,              // CS pin
      .queue_size=7,                      // We want to be able to queue 7 transactions at a time
    };

    //Attach the SENS1 to the SPI bus
    ret=spi_bus_add_device(HOST, &devcfg, &sens1);
    ESP_ERROR_CHECK(ret);
  }

  ram = malloc(sizeof(uint16_t)*DRAM_CNT_HALFWORD);

  printf("[SENS0] ADDR: 0x%lx\n", _spi_read_trans(sens0, LSM6DSM_WHO_AM_I, BYTE));
  printf("[SENS1] ADDR: 0x%lx\n", _spi_read_trans(sens1, LSM6DSM_WHO_AM_I, BYTE));

  _setup_sensor(sens0);
  _setup_sensor(sens1);

  gpio_set_direction(PIN_SEN0_INT1, GPIO_MODE_INPUT);
  gpio_pulldown_en(PIN_SEN0_INT1);
  gpio_pullup_dis(PIN_SEN0_INT1);
  gpio_set_intr_type(PIN_SEN0_INT1, GPIO_INTR_POSEDGE);

  gpio_set_direction(PIN_SEN1_INT1, GPIO_MODE_INPUT);
  gpio_pulldown_en(PIN_SEN1_INT1);
  gpio_pullup_dis(PIN_SEN1_INT1);
  gpio_set_intr_type(PIN_SEN1_INT1, GPIO_INTR_POSEDGE);

  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
  
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
  gpio_isr_handler_add(PIN_SEN0_INT1, gpio_isr_handler, (void*) PIN_SEN0_INT1);
  gpio_isr_handler_add(PIN_SEN1_INT1, gpio_isr_handler, (void*) PIN_SEN1_INT1);

  for(;;) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("rd=%u, wr=%u\n", ram_rd_idx, ram_wr_idx);
    
    while(ram_rd_idx != ram_wr_idx) {
      printf("[%u] %6hd\n", (uint16_t) ram[ram_rd_idx]&0x3, (short) ram[ram_rd_idx]&~0x3);
      ram_rd_idx = (ram_rd_idx == DRAM_CNT_HALFWORD) ? 0 : ram_rd_idx+1;
    }
  }
}

