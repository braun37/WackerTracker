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

#define PIN_S_ONE_INT1 32
#define PIN_S_TWO_INT1 33

#define PARALLEL_LINES 16

//******************************
// Sensor Data Struct
//******************************
#define NUM_ELEM_BLOCK 220

typedef struct _s_output{
  uint16_t ang_x, ang_y, ang_z;
  uint16_t lin_x, lin_y, lin_z;
} s_output;

typedef struct _s_block{
  uint8_t numel;
  s_output *elements;
} s_block;

void _setup_s_block(s_block *s_blk) {
  s_blk->elements = malloc(sizeof(s_output)*NUM_ELEM_BLOCK);
}

// Sensor Blocks
s_block s_one_block = {0, NULL};
s_block s_two_block = {0, NULL};
s_block s_thr_block = {0, NULL};

//******************************
// BLE Setup
//******************************
char *CLIENT_TAG = "BLE Client Scan";
char *SERVER_TAG= "BLE Server Advertise";
uint8_t ble_addr_type;
void ble_app_scan(void);
void ble_app_advertise(void);

static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    printf("Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
    return 0;
}

static int device_read_sensor1(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    //os_mbuf_append(ctxt->om, "X", strlen("X"));
    //os_mbuf_append(ctxt->om, &x_arr, sizeof(x_arr));
   
    // Add all the elements of the data structure into the buffer 
    //for(int i=0; i<s_one_block.numel; i++) {
    //  os_mbuf_append(ctxt->om, &(s_one_block.elements[i]), sizeof(s_one_block.elements[i]));
    //}
    int rc = os_mbuf_append(ctxt->om, s_one_block.elements, sizeof(*s_one_block.elements)*s_one_block.numel);
    assert(rc == 0);

    // Clear the number of elements
    //s_one_block.numel--;

    return 0;
}

static int device_read_sensor1_numElements(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, &(s_one_block.numel), sizeof(s_one_block.numel));

    return 0;
}

static int device_read_sensor2(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    //os_mbuf_append(ctxt->om, "X", strlen("X"));
    //os_mbuf_append(ctxt->om, &x_arr, sizeof(x_arr));
   
    // Add all the elements of the data structure into the buffer 
    for(int i=0; i<s_two_block.numel; i++) {
      os_mbuf_append(ctxt->om, &(s_two_block.elements[i]), sizeof(s_two_block.elements[i]));
    }

    // Clear the number of elements
    s_two_block.numel = 0;

    return 0;
}

static int device_read_sensor2_numElements(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, &(s_two_block.numel), sizeof(s_two_block.numel));

    return 0;
}

static int device_read_sensor3(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    //os_mbuf_append(ctxt->om, "X", strlen("X"));
    //os_mbuf_append(ctxt->om, &x_arr, sizeof(x_arr));
   
    // Add all the elements of the data structure into the buffer 
    for(int i=0; i<s_thr_block.numel; i++) {
      os_mbuf_append(ctxt->om, &(s_thr_block.elements[i]), sizeof(s_thr_block.elements[i]));
    }

    // Clear the number of elements
    s_thr_block.numel = 0;

    return 0;
}

static int device_read_sensor3_numElements(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, &(s_thr_block.numel), sizeof(s_thr_block.numel));

    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),                 //Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xFEF4),           //Define UUID for reading
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read_sensor1},
         {.uuid = BLE_UUID16_DECLARE(0xFEF5),
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read_sensor1_numElements},
         {.uuid = BLE_UUID16_DECLARE(0xFEF6),
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read_sensor2},
         {.uuid = BLE_UUID16_DECLARE(0xFEF7),
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read_sensor2_numElements},
         {.uuid = BLE_UUID16_DECLARE(0xFEF8),
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read_sensor3},
         {.uuid = BLE_UUID16_DECLARE(0xFEF9),
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read_sensor3_numElements},
         {.uuid = BLE_UUID16_DECLARE(0xDEAD),           //Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},
         {0}}},
    {0}
};

// BLE event handling
static int ble_gap_event_client(struct ble_gap_event *event, void *arg)
{
    struct ble_hs_adv_fields fields;

    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC:
        ESP_LOGI("GAP", "GAP EVENT DISCOVERY");
        ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (fields.name_len > 0)
        {
            printf("Name: %.*s\n", fields.name_len, fields.name);
        }
        break;
    default:
        break;
    }
    return 0;
}

static int ble_gap_event_server(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_server, NULL);
}

void ble_app_scan(void)
{
    printf("Start scanning ...\n");

    struct ble_gap_disc_params disc_params;
    disc_params.filter_duplicates = 1;
    disc_params.passive = 0;
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    ble_gap_disc(ble_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event_client, NULL);
}

void ble_app_on_sync_server(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The application
void ble_app_on_sync_client(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_scan();                          
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

void run_client(void){
    ble_svc_gap_device_name_set("ESP-32-BLE"); 
    ble_svc_gap_init();                       
    ble_hs_cfg.sync_cb = ble_app_on_sync_client;    
    nimble_port_freertos_init(host_task);   //run thread
}

void run_server(void){
    ble_svc_gap_device_name_set("BLE-Server"); 
    ble_svc_gap_init();                       
    ble_svc_gatt_init();                    
    ble_gatts_count_cfg(gatt_svcs);        
    ble_gatts_add_svcs(gatt_svcs);        
    ble_hs_cfg.sync_cb = ble_app_on_sync_server; 
    nimble_port_freertos_init(host_task); 
}

//void app_main()
//{
//    nvs_flash_init();                           
//    nimble_port_init();                        
//    
//    //BLE Client
//    //run_client();
//    
//    //BLE Server
//    run_server();
//}

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

uint16_t _spi_read_trans(spi_device_handle_t spi, const uint8_t addr, const int len)
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

  return *(uint16_t*)t.rx_data;
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

void _fifo_read(spi_device_handle_t spi, s_block *blk) {
  //assert(!blk->numel);
  blk->numel = (_spi_read_trans(spi, LSM6DSM_FIFO_STATUS1, HALFWORD) & 0x7FFF)/6;

  for(int i=0; i<blk->numel; i++) {
    (blk->elements)[i].ang_x = _spi_read_trans(spi, LSM6DSM_FIFO_DATA_OUT_L, HALFWORD);
    (blk->elements)[i].ang_y = _spi_read_trans(spi, LSM6DSM_FIFO_DATA_OUT_L, HALFWORD);
    (blk->elements)[i].ang_z = _spi_read_trans(spi, LSM6DSM_FIFO_DATA_OUT_L, HALFWORD);
    (blk->elements)[i].lin_x = _spi_read_trans(spi, LSM6DSM_FIFO_DATA_OUT_L, HALFWORD);
    (blk->elements)[i].lin_y = _spi_read_trans(spi, LSM6DSM_FIFO_DATA_OUT_L, HALFWORD);
    (blk->elements)[i].lin_z = _spi_read_trans(spi, LSM6DSM_FIFO_DATA_OUT_L, HALFWORD);
  }
}

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
  uint32_t gpio_pin = (uint32_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_pin, NULL);
}

spi_device_handle_t s_one;
spi_device_handle_t s_two;
spi_device_handle_t s_thr;

static void gpio_task_example(void* arg) {
  uint32_t io_pin;
  for(;;) {
    if(xQueueReceive(gpio_evt_queue, &io_pin, portMAX_DELAY)) {
        if(gpio_get_level(io_pin) == 0) continue;

        switch(io_pin) {
          case 32: 
            printf("SENSOR1 INTERRUPT\n");
            _fifo_read(s_one, &s_one_block);
            printf("numel=%u\n", s_one_block.numel);
            break;
          case 33: 
            printf("SENSOR2 INTERRUPT\n");
            _fifo_read(s_two, &s_two_block);
            printf("numel=%u\n", s_two_block.numel);
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
      .clock_speed_hz=1*1000*1000,        // Clock out at 5 MHz
      .mode=0,                            // SPI mode 0
      .spics_io_num=PIN_CS0,              // CS pin
      .queue_size=7,                      // We want to be able to queue 7 transactions at a time
    };

    //Attach the SENS0 to the SPI bus
    ret=spi_bus_add_device(HOST, &devcfg, &s_one);
    ESP_ERROR_CHECK(ret);
  }

  {
    spi_device_interface_config_t devcfg = {
      .clock_speed_hz=1*1000*1000,        // Clock out at 5 MHz
      .mode=0,                            // SPI mode 0
      .spics_io_num=PIN_CS1,              // CS pin
      .queue_size=7,                      // We want to be able to queue 7 transactions at a time
    };

    //Attach the SENS1 to the SPI bus
    ret=spi_bus_add_device(HOST, &devcfg, &s_two);
    ESP_ERROR_CHECK(ret);
  }

  // Setup all the sensor blocks in the DRAM
  _setup_s_block(&s_one_block);
  _setup_s_block(&s_two_block);
  _setup_s_block(&s_thr_block);

  _setup_sensor(s_one);
  _setup_sensor(s_two);

  gpio_set_direction(PIN_S_ONE_INT1, GPIO_MODE_INPUT);
  gpio_pulldown_en(PIN_S_ONE_INT1);
  gpio_pullup_dis(PIN_S_ONE_INT1);
  gpio_set_intr_type(PIN_S_ONE_INT1, GPIO_INTR_POSEDGE);

  gpio_set_direction(PIN_S_TWO_INT1, GPIO_MODE_INPUT);
  gpio_pulldown_en(PIN_S_TWO_INT1);
  gpio_pullup_dis(PIN_S_TWO_INT1);
  gpio_set_intr_type(PIN_S_TWO_INT1, GPIO_INTR_POSEDGE);

  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
  
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
  gpio_isr_handler_add(PIN_S_ONE_INT1, gpio_isr_handler, (void*) PIN_S_ONE_INT1);
  gpio_isr_handler_add(PIN_S_TWO_INT1, gpio_isr_handler, (void*) PIN_S_TWO_INT1);

  //for(;;) {
  //  vTaskDelay(1000 / portTICK_PERIOD_MS);

  //  if(s_one_block.num) {
  //    for(int i=0; i<s_one_block.num; i++) {
  //      printf("[1] [%6hd, %6hd, %6hd] (%6hd, %6hd, %6hd)\n", 
  //          (short) s_one_block.elements[i].ang_x,
  //          (short) s_one_block.elements[i].ang_y,
  //          (short) s_one_block.elements[i].ang_z,
  //          (short) s_one_block.elements[i].lin_x,
  //          (short) s_one_block.elements[i].lin_y,
  //          (short) s_one_block.elements[i].lin_z);
  //    }
  //    s_one_block.num = 0;
  //  }

  //  if(s_two_block.num) {
  //    for(int i=0; i<s_two_block.num; i++) {
  //      printf("[2] [%6hd, %6hd, %6hd] (%6hd, %6hd, %6hd)\n", 
  //          (short) s_two_block.elements[i].ang_x,
  //          (short) s_two_block.elements[i].ang_y,
  //          (short) s_two_block.elements[i].ang_z,
  //          (short) s_two_block.elements[i].lin_x,
  //          (short) s_two_block.elements[i].lin_y,
  //          (short) s_two_block.elements[i].lin_z);
  //    }
  //    s_two_block.num = 0;
  //  }

  //  if(s_thr_block.num) {
  //    for(int i=0; i<s_one_block.num; i++) {
  //      printf("[3] [%6hd, %6hd, %6hd] (%6hd, %6hd, %6hd)\n", 
  //          (short) s_thr_block.elements[i].ang_x,
  //          (short) s_thr_block.elements[i].ang_y,
  //          (short) s_thr_block.elements[i].ang_z,
  //          (short) s_thr_block.elements[i].lin_x,
  //          (short) s_thr_block.elements[i].lin_y,
  //          (short) s_thr_block.elements[i].lin_z);
  //    }
  //    s_thr_block.num = 0;
  //  }
  //}

  nvs_flash_init();                           
  nimble_port_init();                        
  
  //BLE Client
  //run_client();
  
  //BLE Server
  run_server();
}

