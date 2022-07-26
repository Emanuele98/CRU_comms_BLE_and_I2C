
#include "driver/gpio.h"
#include "include/pru_hw.h"

/* 
 * All local readings (on PTU) are not calibrated. Therefore, you must provide a
 * way to convert digital input data to the respective reading types. For example,
 * in the `read_temperature` function, you could convert raw digital data to real
 * temperature by using an intercept and a slope.
 *  
 * It is best to define such values as preprocessor definitions using the #define
 * directive.
 */

static const char* TAG = "HARDWARE";

/**
 * @brief read sensor data
 *
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
float i2c_read_voltage_sensor(void)
{
    int ret;
    uint8_t first_byte, second_byte;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, V_A_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, V_REGISTER_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(ret);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, V_A_SENSOR_ADDR  << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &first_byte, ACK_VAL);
    i2c_master_read_byte(cmd, &second_byte, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(ret);

    //printf("first byte: %02x\n", byte_1);
    //printf("second byte : %02x\n", byte_2);
    float value = (int16_t)(first_byte << 8 | second_byte) * 0.00125;
    //printf("sensor val: %.02f [V]\n", value);

    return value;
}

float i2c_read_current_sensor(void)
{
    int ret;
    uint8_t first_byte, second_byte;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, V_A_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, A_REGISTER_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(ret);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, V_A_SENSOR_ADDR  << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &first_byte, ACK_VAL);
    i2c_master_read_byte(cmd, &second_byte, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(ret);

    //printf("first byte: %02x\n", first_byte);
    //printf("second byte : %02x\n", second_byte);
    float value = (int16_t)(first_byte << 8 | second_byte) * 0.0000025 / 0.012;
    //printf("sensor val: %.02f [A]\n", value);

    return value;
}


float i2c_read_temperature_sensor(void)
{
    int ret;
    uint8_t first_byte, second_byte;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, T_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, T_REGISTER_ADDR, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(ret);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, T_SENSOR_ADDR  << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &first_byte, ACK_VAL);
    i2c_master_read_byte(cmd, &second_byte, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(ret);

    //printf("first byte: %02x\n", first_byte);
    //printf("second byte : %02x\n", second_byte);
    float value = (int16_t)(first_byte << 4 | second_byte >> 4) * 0.0625 ;
    //printf("sensor val: %.02f [°C]\n", value);

    return value;
}

//? do the mean value??

/** 
 * @brief Mean function for ADC measurements
 * @details Handles a few samples from a specified ADC channel and averages
 *          it on the number of samples.
 * 
 * @param pin_number Pin or channel number used for measurements
 * @param n_samples  Number of samples needed
 * 
 * @return The average value (in bits) over the number of samples
*/
static int mean_adc(uint8_t pin_number, uint32_t n_samples)
{
    int mean = 0;
    for (int i=0; i!=n_samples; i++)
    {
        mean += adc1_get_raw(pin_number);
    }
    return (int)(mean/n_samples);
}

void enable_power_output(void)
{
    enabled = 1;
    gpio_set_level(ENABLE_POWER_OUT_PIN, 1);
}

void disable_power_output(void)
{
    enabled = 0;
    gpio_set_level(ENABLE_POWER_OUT_PIN, 0);
}

uint8_t is_power_enabled(void)
{
    return enabled;
}