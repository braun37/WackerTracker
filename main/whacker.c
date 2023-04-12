#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_nimble_hci.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/event_groups.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"
#include "whacker.h"


char *CLIENT_TAG = "BLE Client Scan";
char *SERVER_TAG= "BLE Server Advertise";
uint8_t ble_addr_type;
void ble_app_scan(void);
void ble_app_advertise(void);

//int read_flag = 0;
//int x_arr[] = {1, 2, 3};

//short int* p = test3;
//int index = 0
//write data

sensor_output* sensor1_pointer = testObj.test1;
sensor_output* sensor2_pointer = testObj.test1;
sensor_output* sensor3_pointer = testObj.test1;

static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    printf("Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
    return 0;
}

//read data
static int device_read_sensor1(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    //os_mbuf_append(ctxt->om, "X", strlen("X"));
    //os_mbuf_append(ctxt->om, &x_arr, sizeof(x_arr));

    //maybe set if statement on if read flag is 0/1 so that
    //when its 1 it knows its mid read and doesnt reset arr
    //short int* p = test3;
    //os_mbuf_append(ctxt->om, p, (sizeof(*(p)) * 6));
    os_mbuf_append(ctxt->om, sensor1_pointer, (sizeof(*(sensor1_pointer))));
    //p += 6;
    sensor1_pointer += 1;

    return 0;
}

static int device_read_sensor1_numElements(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, &(testObj.numElements), sizeof(testObj.numElements));

    return 0;
}

static int device_read_sensor2(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    //os_mbuf_append(ctxt->om, "X", strlen("X"));
    //os_mbuf_append(ctxt->om, &x_arr, sizeof(x_arr));

    //maybe set if statement on if read flag is 0/1 so that
    //when its 1 it knows its mid read and doesnt reset arr
    //short int* p = test3;
    //os_mbuf_append(ctxt->om, p, (sizeof(*(p)) * 6));
    os_mbuf_append(ctxt->om, sensor1_pointer, (sizeof(*(sensor1_pointer))));
    //p += 6;
    sensor1_pointer += 1;

    return 0;
}

static int device_read_sensor2_numElements(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, &(testObj.numElements), sizeof(testObj.numElements));

    return 0;
}

static int device_read_sensor3(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    //os_mbuf_append(ctxt->om, "X", strlen("X"));
    //os_mbuf_append(ctxt->om, &x_arr, sizeof(x_arr));

    //maybe set if statement on if read flag is 0/1 so that
    //when its 1 it knows its mid read and doesnt reset arr
    //short int* p = test3;
    //os_mbuf_append(ctxt->om, p, (sizeof(*(p)) * 6));
    os_mbuf_append(ctxt->om, sensor1_pointer, (sizeof(*(sensor1_pointer))));
    //p += 6;
    sensor1_pointer += 1;

    return 0;
}

static int device_read_sensor3_numElements(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om, &(testObj.numElements), sizeof(testObj.numElements));

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

void app_main()
{
    nvs_flash_init();                           
    nimble_port_init();                        
    
    //BLE Client
    //run_client();
    
    //BLE Server
    run_server();
}