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
#include "max30102.h"
#include "dsps_fir.h"
static const char *TAG = "i2c-simple-example";







#define BLOCK_SIZE           1
#define NUM_TAPS             29

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = BLOCK_SIZE;
fir_f32_t S_red;
fir_f32_t S_ir;
static float firStateF32_ir[BLOCK_SIZE + NUM_TAPS - 1];
static float firStateF32_red[BLOCK_SIZE + NUM_TAPS - 1];


const float firCoeffs32LP[NUM_TAPS] = {
        -0.001542701735,-0.002211477375,-0.003286228748, -0.00442651147,-0.004758632276,
        -0.003007677384, 0.002192312852,  0.01188309677,  0.02637642808,  0.04498152807,
        0.06596207619,   0.0867607221,   0.1044560149,   0.1163498312,   0.1205424443,
        0.1163498312,   0.1044560149,   0.0867607221,  0.06596207619,  0.04498152807,
        0.02637642808,  0.01188309677, 0.002192312852,-0.003007677384,-0.004758632276,
        -0.00442651147,-0.003286228748,-0.002211477375,-0.001542701735
};








#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0))
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

#define CACHE_NUMS 300
#define PPG_DATA_THRESHOLD 100000
uint8_t max30102_int_flag = 0;

float ppg_data_cache_RED[CACHE_NUMS] = {0};
float ppg_data_cache_IR[CACHE_NUMS] = {0};
static int fuck = 0;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    fuck = 1;
}

void app_main(void) {
    dsps_fir_init_f32(&S_ir, firCoeffs32LP, firStateF32_ir, NUM_TAPS);
    dsps_fir_init_f32(&S_red, firCoeffs32LP, firStateF32_red, NUM_TAPS);


    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);


    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *) GPIO_INPUT_IO_0);
    max30102_init();
    uint16_t cache_counter = 0;
    float max30102_data[2], fir_output[2];
    uint32_t io_num;
    while (1) {
        if (fuck) {
            fuck = 0;
            max30102_fifo_read(max30102_data);
//            fir_output[0] = max30102_data[0];
//            fir_output[1] = max30102_data[1];
            dsps_fir_f32_ansi(&S_ir, &max30102_data[0], &fir_output[0], 1);
            dsps_fir_f32_ansi(&S_red, &max30102_data[1], &fir_output[1], 1);

            if ((max30102_data[0] > PPG_DATA_THRESHOLD) && (max30102_data[1] > PPG_DATA_THRESHOLD)) {
                ppg_data_cache_IR[cache_counter] = fir_output[0];
                ppg_data_cache_RED[cache_counter] = fir_output[1];
                cache_counter++;
            } else {
                cache_counter = 0;
            }


            if (cache_counter >= CACHE_NUMS) {
                printf("heart rate %d/min   ", max30102_getHeartRate(ppg_data_cache_IR, CACHE_NUMS));
                printf("o2  %.2f\n", max30102_getSpO2(ppg_data_cache_IR, ppg_data_cache_RED, CACHE_NUMS));
                cache_counter = 0;
            }
        } else {
            vTaskDelay(1);
        }


    }


}
