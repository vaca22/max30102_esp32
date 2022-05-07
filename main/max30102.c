
#include <stdint-gcc.h>

#include <driver/i2c.h>
#include <esp_log.h>
#include "max30102.h"


static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1

#define MAX30102ADDR    (0xae>>1)

static esp_err_t i2c_master_init(void) {
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

static esp_err_t wr_max30102_one_data(uint8_t saddress, uint8_t w_data) {
    uint8_t write_buf[2] = {saddress, w_data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MAX30102ADDR,
                                      write_buf, 2,
                                      I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

static esp_err_t rd_max30102_data(uint8_t saddress, uint8_t *r_data, uint8_t data_size) {


    return i2c_master_write_read_device(I2C_MASTER_NUM, MAX30102ADDR,
                                        &saddress, 1,
                                        r_data, data_size,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}


void max30102_i2c_write(uint8_t reg_adder, uint8_t data) {
    uint8_t transmit_data[2];
    transmit_data[0] = reg_adder;
    transmit_data[1] = data;
    i2c_master_write_to_device(I2C_MASTER_NUM, MAX30102ADDR,
                               transmit_data, 2,
                               I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

}

void max30102_i2c_read(uint8_t reg_adder, uint8_t *pdata, uint8_t data_size) {
    i2c_master_write_read_device(I2C_MASTER_NUM, MAX30102ADDR,
                                 &reg_adder, 1,
                                 pdata, data_size,
                                 I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

}

void max30102_init(void) {
    i2c_master_init();
    vTaskDelay(1);
    uint8_t data;

    max30102_i2c_write(MODE_CONFIGURATION,0x40);  //reset the device

    vTaskDelay(5);

    max30102_i2c_write(INTERRUPT_ENABLE1,0xE0);
    max30102_i2c_write(INTERRUPT_ENABLE2,0x00);  //interrupt enable: FIFO almost full flag, new FIFO Data Ready,
    //                   ambient light cancellation overflow, power ready flag,
    //						    		internal temperature ready flag

    max30102_i2c_write(FIFO_WR_POINTER,0x00);
    max30102_i2c_write(FIFO_OV_COUNTER,0x00);
    max30102_i2c_write(FIFO_RD_POINTER,0x00);   //clear the pointer

    max30102_i2c_write(FIFO_CONFIGURATION,0x4F); //FIFO configuration: sample averaging(1),FIFO rolls on full(0), FIFO almost full value(15 empty data samples when interrupt is issued)

    max30102_i2c_write(MODE_CONFIGURATION,0x03);  //MODE configuration:SpO2 mode

    max30102_i2c_write(SPO2_CONFIGURATION,0x2A); //SpO2 configuration:ACD resolution:15.63pA,sample rate control:200Hz, LED pulse width:215 us

    max30102_i2c_write(LED1_PULSE_AMPLITUDE,0x2f);	//IR LED
    max30102_i2c_write(LED2_PULSE_AMPLITUDE,0x2f); //RED LED current

    max30102_i2c_write(TEMPERATURE_CONFIG,0x01);   //temp

    max30102_i2c_read(INTERRUPT_STATUS1,&data,1);
    max30102_i2c_read(INTERRUPT_STATUS2,&data,1);  //clear status
}


void max30102_fifo_read(float *output_data)
{
    uint8_t receive_data[6];
    uint32_t data[2];
    max30102_i2c_read(FIFO_DATA,receive_data,6);
    data[0] = ((receive_data[0]<<16 | receive_data[1]<<8 | receive_data[2]) & 0x03ffff);
    data[1] = ((receive_data[3]<<16 | receive_data[4]<<8 | receive_data[5]) & 0x03ffff);
    *output_data = data[0];
    *(output_data+1) = data[1];



}

uint16_t max30102_getHeartRate(float *input_data,uint16_t cache_nums)
{
    float input_data_sum_aver = 0;
    uint16_t i,temp=0;


    for(i=0;i<cache_nums;i++)
    {
        input_data_sum_aver += *(input_data+i);
    }
    input_data_sum_aver = input_data_sum_aver/cache_nums;
    for(i=0;i<cache_nums;i++)
    {
        if((*(input_data+i)>input_data_sum_aver)&&(*(input_data+i+1)<input_data_sum_aver))
        {
            temp = i;
            break;
        }
    }
    i++;
    for(;i<cache_nums;i++)
    {
        if((*(input_data+i)>input_data_sum_aver)&&(*(input_data+i+1)<input_data_sum_aver))
        {
            temp = i - temp;
            break;
        }
    }
    if((temp>14)&&(temp<100))
    {
        return 3000/temp;
    }
    else
    {
        return 0;
    }
}

float max30102_getSpO2(float *ir_input_data,float *red_input_data,uint16_t cache_nums)
{
    float ir_max=*ir_input_data,ir_min=*ir_input_data;
    float red_max=*red_input_data,red_min=*red_input_data;
    float R;
    uint16_t i;
    for(i=1;i<cache_nums;i++)
    {
        if(ir_max<*(ir_input_data+i))
        {
            ir_max=*(ir_input_data+i);
        }
        if(ir_min>*(ir_input_data+i))
        {
            ir_min=*(ir_input_data+i);
        }
        if(red_max<*(red_input_data+i))
        {
            red_max=*(red_input_data+i);
        }
        if(red_min>*(red_input_data+i))
        {
            red_min=*(red_input_data+i);
        }
    }

    R=((ir_max+ir_min)*(red_max-red_min))/((red_max+red_min)*(ir_max-ir_min));
    return ((-45.060)*R*R + 30.354*R + 94.845);
}
