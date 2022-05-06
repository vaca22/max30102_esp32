#ifndef __MAX30102_H
#define __MAX30102_H





#define INTERRUPT_STATUS1 0x00
#define INTERRUPT_STATUS2 0x01
#define INTERRUPT_ENABLE1 0x02
#define INTERRUPT_ENABLE2 0x03

#define FIFO_WR_POINTER 0x04
#define FIFO_OV_COUNTER 0x05
#define FIFO_RD_POINTER 0x06
#define FIFO_DATA 0x07

#define FIFO_CONFIGURATION 0x08
#define MODE_CONFIGURATION 0x09
#define SPO2_CONFIGURATION 0x0A
#define LED1_PULSE_AMPLITUDE 0x0C
#define LED2_PULSE_AMPLITUDE 0x0D

#define MULTILED1_MODE 0x11
#define MULTILED2_MODE 0x12

#define TEMPERATURE_INTEGER 0x1F
#define TEMPERATURE_FRACTION 0x20
#define TEMPERATURE_CONFIG 0x21

#define VERSION_ID 0xFE
#define PART_ID 0xFF

void max30102_init(void);
void max30102_fifo_read(float *data);
void max30102_i2c_read(uint8_t reg_adder,uint8_t *pdata, uint8_t data_size);
uint16_t max30102_getHeartRate(float *input_data,uint16_t cache_nums);
float max30102_getSpO2(float *ir_input_data,float *red_input_data,uint16_t cache_nums);
#endif /* __MAX30102_H */
