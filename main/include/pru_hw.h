#ifndef __PRU_HW_H__
#define __PRU_HW_H__

#include <math.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "esp_log.h"


#include "pru_bsp.h"

/* I2C MASTER VALUES */
#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define V_A_SENSOR_ADDR 0x40  /*!< slave address for voltage and current sensor */
#define T_SENSOR_ADDR 0x4A    /*!< slave address for temperature sensor */
#define V_REGISTER_ADDR 0x02  /* bus voltage register address */
#define A_REGISTER_ADDR 0x01  /* shunt register address */
#define T_REGISTER_ADDR 0x00  /* temperature register address */


#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

/* Critical error values (arbitrary values) */
#define LOCAL_OTP                    60                // Local fault indicator for overtemperature (in celsius)
#define LOCAL_OVP                    10000             // Local fault indicator for overvoltage (in mV)
#define LOCAL_OCP                    ITX_ABS_MAX       // Local fault indicator for overcurrent (in mA)


/* Keeps power output state in memory */
uint8_t enabled;

/* Semaphore used to protect against I2C reading simultaneously */
SemaphoreHandle_t i2c_sem;

float i2c_read_voltage_sensor(void);
float i2c_read_current_sensor(void);
float i2c_read_temperature_sensor(void);

/** 
 * @brief Power output state function (to ON)
 * @details Changes the current state of the power output to ON.
*/
void enable_power_output(void);

/** 
 * @brief Power output state function (to OFF)
 * @details Changes the current state of the power output to OFF.
*/
void disable_power_output(void);

/** 
 * @brief Boolean function for power output state
 * @details Checks in which state the power output is in.
 * 
 * @return 0 if the power output is closed, 1 if it is open
*/
uint8_t is_power_enabled(void);

#endif