
#ifndef __AIRFUEL_STANDARD_H__
#define __AIRFUEL_STANDARD_H__

// PRU CONTROL
// Enables byte
#define ENABLE_PRU_OUTPUT  (1 << 7)
#define DISABLE_PRU_OUTPUT (0 << 7)

#define ENABLE_PRU_CHARGE_INDICATOR  (1 << 6)
#define DISABLE_PRU_CHARGE_INDICATOR (0 << 6)

#define MAXIMUM_POWER   (00 << 4)
#define TWO_THIRD_POWER (01 << 4)
#define ONE_THIRD_POWER (10 << 4)
#define MINIMUM_POWER   (11 << 4)

#define NO_PRU_EMULATION (000 << 1)
#define EMULATE_CAT_1    (001 << 1)
#define EMULATE_CAT_2    (010 << 1)
#define EMULATE_CAT_3    (011 << 1)
#define EMULATE_CAT_4    (100 << 1)
#define EMULATE_CAT_5    (101 << 1)
#define EMULATE_CAT_6    (110 << 1)

// Permission byte
#define PERMITTED                   0b00000000
#define PERMITTED_WITH_WAITING_TIME 0b00000001
#define DENIED_CROSS_CONNECTION     0b10000000
#define DENIED_AVAILABLE_POWER      0b10000001
#define DENIED_NUMBER_OF_DEVICE     0b10000010
#define DENIED_CLASS_SUPPORT        0b10000011
#define DENIED_HIGH_TEMPERATURE     0b10000100

#endif