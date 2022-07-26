#ifndef __PRU_BSP_H__
#define __PRU_BSP_H__

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/dac.h"


/* Arbitrary pin values */
#define IOUT_PIN                     ADC1_CHANNEL_5        /* GPIO33 */
#define VOUT_PIN                     ADC1_CHANNEL_4        /* GPIO32 */
#define TEMPERATURE_PIN              ADC1_CHANNEL_3        /* GPIO39 */
#define ENABLE_POWER_OUT_PIN         GPIO_NUM_22           /* GPIO22 */

#endif
