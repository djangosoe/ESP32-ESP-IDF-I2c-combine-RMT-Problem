#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"

#include <stdint.h>
#include <NeoPixelBus.h>

//#define _I2C_NUMBER(num) I2C_NUM_##num
//#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                                     /*!< Data buffer length of test buffer */ //default 512
#define RW_TEST_LENGTH 128                                  /*!< Data length for r/w test, [0,DATA_LENGTH] */ //default 128

#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS             /*!< ESP32 slave address, you can set any 7bit value */
#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM 0                                     /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

static const uint8_t Strip0Pin = 27;
static const uint8_t Strip1Pin = 16;
static const uint8_t Strip2Pin = 17;
static const uint8_t Strip3Pin = 18;
static const uint8_t Strip4Pin = 19;
static const uint8_t Strip5Pin = 4;

extern "C" {
    void app_main(void);
}

/**
 * @brief i2c slave initialization
 */
static esp_err_t i2c_slave_init()
{
    i2c_port_t i2c_slave_port = (i2c_port_t)I2C_SLAVE_NUM;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = (gpio_num_t)I2C_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = (gpio_num_t)I2C_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    return i2c_driver_install(i2c_slave_port, conf_slave.mode,
                              I2C_SLAVE_RX_BUF_LEN,
                              I2C_SLAVE_TX_BUF_LEN, 0);
}

struct MyConfig {
    uint16_t pixelCount;
    uint16_t stripCount;
}; 

void show_data_loop(void *pvParameter)
{
    struct MyConfig *MyConfigPtr;
    MyConfigPtr = (MyConfig*)pvParameter;


    
    NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt0Ws2812xMethod> strip0(60, Strip0Pin);
    NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt1Ws2812xMethod> strip1(60, Strip1Pin);
    NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt2Ws2812xMethod> strip2(60, Strip2Pin);
    NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt3Ws2812xMethod> strip3(60, Strip3Pin);
    NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt4Ws2812xMethod> strip4(60, Strip4Pin);
    NeoPixelBus<NeoGrbFeature, NeoEsp32Rmt5Ws2812xMethod> strip5(60, Strip5Pin);

    strip0.Begin();
    strip1.Begin();
    strip2.Begin();
    strip3.Begin();
    strip4.Begin();
    strip5.Begin();

    int count = 0;
    while (1) 
    {  
        for (int s =0; s < MyConfigPtr->stripCount; s++)
        {
            for (int i =0; i < MyConfigPtr->pixelCount; i++)
            {
                if(s == 0)
                {
                    strip0.SetPixelColor(i, {0,0,0});
                }
                else if(s == 1)
                {
                    strip1.SetPixelColor(i, {0,0,0});
                }
                else if(s == 2)
                {
                    strip2.SetPixelColor(i, {0,0,0});
                }
                else if(s == 3)
                {
                    strip3.SetPixelColor(i, {0,0,0});
                }
                else if(s == 4)
                {
                    strip4.SetPixelColor(i, {0,0,0});
                }
                else if(s == 5)
                {
                    strip5.SetPixelColor(i, {0,0,0});
                }
            }
        } 

        strip0.SetPixelColor(count % MyConfigPtr->pixelCount, {0,0,100});
        strip1.SetPixelColor(count % MyConfigPtr->pixelCount, {0,0,100});
        strip2.SetPixelColor(count % MyConfigPtr->pixelCount, {0,0,100});
        strip3.SetPixelColor(count % MyConfigPtr->pixelCount, {0,0,100});
        strip4.SetPixelColor(count % MyConfigPtr->pixelCount, {0,0,100});
        strip5.SetPixelColor(count % MyConfigPtr->pixelCount, {0,0,100});


        strip0.Show();
        strip1.Show();
        strip2.Show();
        strip3.Show();
        strip4.Show();
        strip5.Show();

        count+=1;
        vTaskDelay((33) / portTICK_RATE_MS);
    }
}

void receive_data_loop(void *pvParameter)
{   
    ESP_ERROR_CHECK(i2c_slave_init());
    
    uint8_t byte = 0;

    while (1) {     
        i2c_slave_read_buffer(I2C_NUM_0, &byte, 1, portMAX_DELAY);
        //printf("Data Received: %d = ", byte);
    }
    vTaskDelete( NULL );
}


void app_main()
{
    printf(" LedWall: Start Main loop. \n");


    static struct MyConfig MyConfig;
    MyConfig.stripCount = 6;
    MyConfig.pixelCount = 60;

    printf("Initial StripCount: %d \n", MyConfig.stripCount);
    printf("Initial PixelCount: %d \n", MyConfig.pixelCount);

    xTaskCreatePinnedToCore(&receive_data_loop, "receive_data_loop", 2048, &MyConfig, 10, NULL, 1);
    xTaskCreatePinnedToCore(&show_data_loop, "show_data_loop", 2048, &MyConfig, 10, NULL,0);
}
