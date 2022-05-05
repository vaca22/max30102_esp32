/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MAX30102ADDR    (0xae>>1)

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}







static esp_err_t wr_max30102_one_data(uint8_t address,uint8_t saddress,uint8_t w_data )
{
    uint8_t write_buf[2] = {saddress,w_data};
    return  i2c_master_write_to_device(I2C_MASTER_NUM, address,write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

static esp_err_t rd_max30102_one_data(uint8_t address,uint8_t saddress,uint8_t *r_data)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, address, &saddress, 1, r_data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

void app_main(void)
{
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    uint8_t rda;


    while (1){
        ESP_ERROR_CHECK(wr_max30102_one_data(MAX30102ADDR,0x09,0x0b));
        ESP_ERROR_CHECK(rd_max30102_one_data(MAX30102ADDR,0xff,&rda));
        ESP_LOGE("fuck","max30102 ID =%d\n",rda);

        ESP_ERROR_CHECK(wr_max30102_one_data(MAX30102ADDR,0x08,0x43));

        ESP_ERROR_CHECK(wr_max30102_one_data(MAX30102ADDR,0x21,0x01));
        vTaskDelay(5);
        ESP_ERROR_CHECK(rd_max30102_one_data(MAX30102ADDR,0x1F,&rda));
        ESP_LOGE("fuck","temp1=%d\n",rda);
        int temp1=rda;
        ESP_ERROR_CHECK(rd_max30102_one_data(MAX30102ADDR,0x20,&rda));
        ESP_LOGE("fuck","temp2=%d\n",rda);
        int temp2=rda;
        float temp=temp1+(temp2*0.0625);
        ESP_LOGE("fuck","Temperature=%.4f\n",temp);
        vTaskDelay(100);
    }






}
