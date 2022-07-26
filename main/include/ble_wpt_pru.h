#ifndef BLE_WPT_H__
#define BLE_WPT_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//watchdog
#include "esp_task_wdt.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/ble.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/util/util.h"

/*
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
*/
#include "airfuel_integration.h"

//timers
TimerHandle_t dynamic_t_handle, alert_t_handle;

#define N_BYTES_IN_UUID 16
#define COMPANY_ID                      0x00FF                                 /**< Airfuel company id. Is passed in advertising data. */

/******************************************************************/

/**@brief   Macro for defining all characteristic payload instances */
#define WPT_DYNAMIC_PAYLOAD_DEF(_value1_name)                                                       \
wpt_dynamic_payload_t _value1_name;

#define WPT_STATIC_PAYLOAD_DEF(_value2_name)                                                        \
wpt_static_payload_t _value2_name;

#define WPT_ALERT_PAYLOAD_DEF(_value3_name)                                                         \
wpt_alert_payload_t _value3_name;                  

#define WPT_PTU_STATIC_PAYLOAD_DEF(_value4_name)                                                    \
wpt_ptu_static_payload_t _value4_name;

#define WPT_CONTROL_PAYLOAD_DEF(_value5_name)                                                       \
wpt_control_payload_t _value5_name; 



#define WPT_SERVICE_UUID                                       0xFFFE

// WPT_CHARGING_XXX_UUID Airfuel characteristics
#define WPT_CHARGING_BASE_UUID                                 0xE670

#define WPT_CHARGING_PRU_CONTROL_UUID                          0xE670
#define WPT_CHARGING_PTU_STATIC_UUID                           0xE671
#define WPT_CHARGING_PRU_ALERT_UUID                            0xE672
#define WPT_CHARGING_PRU_STATIC_UUID                           0xE673
#define WPT_CHARGING_PRU_DYNAMIC_UUID                          0xE674

// XXX_CHAR_SIZE characteristic sizes.
#define PRU_CONTROL_CHAR_SIZE                                  5
#define PTU_STATIC_CHAR_SIZE                                   17
#define PRU_STATIC_CHAR_SIZE                                   20
#define PRU_DYNAMIC_CHAR_SIZE                                  20
#define PRU_ALERT_CHAR_SIZE                                    6

// WPT_READ_XXX attribute permissions (capabilities).
#define WPT_READ_ONLY										   1
#define WPT_READ_WRITE										   2
#define WPT_READ_NOTIFY										   3
#define WPT_READ_WRITE_NOTIFY		                           4


/*******************************************************************/
/**                   LOCAL PRU CONFIGURATIONS                    **/
/*******************************************************************/

#define OPTIONAL_FIELDS_STAT            0x00                                   /**< Optional fields requiered. */
#define PROTOCOL_REVISION               0x00                                   /**< Protocol revision of PRU. */
#define PRU_CATEGORY                    0x00                                   /**< Category of PRU. */
#define PRU_INFORMATION                 0x00                                   /**< Information relative to PRU setup. */
#define PRU_HARD_REVISION               0x00                                   /**< Hardware revision of PRU. */
#define PRU_FIRM_REVISION               0x00                                   /**< Firmware revision of PRU. */
#define PRECT_MAXIMUM                   0x00                                   /**< Maximum Prect allowed for PRU. */

#define VRECT_DYN_MINIMUM               10000                                  /**< Minimum Vrect allowerd for PRU. */
#define VRECT_DYN_HIGH                  20000                                  /**< Highest Vrect allowed. */
#define VRECT_DYN_SET                   15000                                  /**< Setting of Vrect value for PRU. */

#define OVER_CURRENT                    3500								   /**< Maximum current tolerated for SAADC measurements. Flow will change in the future to handle such alert. */
#define OVER_VOLTAGE                    40000                                  /**< Maximum voltage tolerated for SAADC measurements. Flow will change in the future to handle such alert. */
#define OVER_TEMPERATURE                600									   /**< Maximum temperature tolerated for SAADC measurements. */


/** @brief Dynamic characteristic structure. This contains elements necessary for dynamic payload. */
typedef struct
{
    uint8_t           optional_fields;    /**< [optional] Optional fields to be added if necessary (1 byte). */
    uint16_t          vrect;              /**< [mandatory] Vrect value  (2 bytes). */
    uint16_t          irect;              /**< [mandatory] Irect value  (2 bytes). */
	uint16_t		  vout;               /**< [optional] Vout value  (2 bytes). */
	uint16_t          iout;               /**< [optional] Iout value  (2 bytes). */
    uint16_t          temp_ratio;         /**< [optional] Temperature ratio calculated from temperature sensor (2 bytes). */
    uint16_t          vrect_min_dyn;      /**< [optional] Minimum Vrect value restriction (2 bytes). */
    uint16_t          vrect_set_dyn;      /**< [optional] Set the desired voltage output for Vrect value (2 bytes). */
    uint16_t          vrect_high_dyn;     /**< [optional] Highest Vrect value restriction (2 bytes). */
	uint8_t	          pru_alert;		  /**< [mandatory] PRU alert field for warnings to send to PTU (1 byte). */
	uint8_t	          tester_command;     /**< [optional] Enable test mode on PTU (1 byte). */
	uint8_t	          dynamic_pru_info;	  /**< [optional] Information reserved for pru dynamic characteristic (1 byte). */
	uint8_t	          RFU;		          /**< [optional] Reserved for future use (1 byte). */
} wpt_dynamic_payload_t;

/** @brief Dynamic characteristic structure. This contains elements necessary for static payload. */
typedef struct
{
    uint8_t           optional_fields;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
    uint8_t           protocol_rev;       /**< [mandatory] Airfuel resonantsupported revision (1 bytes). */
    uint8_t           RFU1;				  /**< [N/A] Reserved for future use (1 bytes). */
	uint8_t		      pru_cat;            /**< [mandatory] Category of PRU (according to Airfuel) (1 bytes). */
	uint8_t           pru_info;           /**< [mandatory] Information regarding capabilities of PRU (1 bytes). */
    uint8_t           hard_rev;           /**< [mandatory] Hardware revision for PRU (1 bytes). */
    uint8_t           firm_rev;           /**< [mandatory] Firmware revision for PRU (1 bytes). */
    uint8_t           prect_max;          /**< [mandatory] Indicates how much power to provide to PRU (1 bytes). */
    uint16_t          vrect_min_stat;     /**< [mandatory] Lowest Vrect value restriction (2 bytes). */
	uint16_t	      vrect_high_stat;	  /**< [mandatory] Highest Vrect value restriction (2 byte). */
	uint16_t	      vrect_set;          /**< [mandatory] Set the desired voltage output for Vrect value (2 byte). */
	uint16_t		  company_id;	      /**< [optional] Company ID for PRU (2 byte). */
	uint32_t	      RFU2;		          /**< [N/A] Reserved for future use (4 byte). */
} wpt_static_payload_t;

/**@brief Alert characteristic structure. This contains elements necessary for alert payload. */
typedef union
{
	struct {
		uint8_t           mode_transition:2;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           wired_charger:1;      /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           charge_complete:1;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           self_protection:1;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           overtemperature:1;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           overcurrent:1;        /**< [mandatory] Defines which optional fields are populated (1 byte). */
		uint8_t           overvoltage:1;        /**< [mandatory] Defines which optional fields are populated (1 byte). */
	};
	uint8_t internal;
} alert_field_t;

typedef struct
{
	alert_field_t     alert_field;
} wpt_alert_payload_t;

/**@brief Control characteristic structure. This contains elements necessary for control payload. */
typedef struct
{
	uint8_t           enables;             /**< [mandatory] PTU turn on, PTU on indication, etc. (1 byte). */
	uint8_t           permission;          /**< [mandatory] Whether PRU is allowd in PTU (1 byte). */
	uint8_t           time_set;            /**< [mandatory] PTU specified set load variation duration (1 byte). */
	uint16_t          RFU;                 /**< [N/A] Undefined (1 byte). */
} wpt_control_payload_t;

/**@brief PTU static characteristic structure. This contains elements necessary for PTU static payload. */
typedef struct
{
	uint8_t           optional_fields;    /**< [mandatory] Defines which optional fields are populated (1 byte). */
    uint8_t           ptu_power;          /**< [mandatory] Power output of PTU (1 bytes). */
    uint8_t           max_impedance;      /**< [DEPRECATED] Unused (1 bytes). */
	uint8_t		      max_load;           /**< [optional] Maximum load_resistance of PTU (1 bytes). */
	uint16_t          RFU1;               /**< [N/A] Undefined (2 bytes). */
    uint8_t           ptu_class;          /**< [mandatory] Hardware class of PTU (1 bytes). */
	uint8_t           hard_rev;           /**< [mandatory] Hardware revision for PTU (1 bytes). */
    uint8_t           firm_rev;           /**< [mandatory] Firmware revision for PTU (1 bytes). */
    uint8_t           protocol_rev;       /**< [mandatory] Airfuel resonant supported revision (1 bytes). */
    uint8_t           max_num_devices;    /**< [mandatory] Maximum number of connected devices the PTU can support (1 bytes). */
	uint16_t	      company_id;	      /**< [mandatory] Company ID for PTU provider (2 byte). */
	uint32_t	      RFU2;		          /**< [N/A] Reserved for future use (4 byte). */
} wpt_ptu_static_payload_t;

/* Instanciation of characteristic values available for Airfuel. */
WPT_DYNAMIC_PAYLOAD_DEF(m_dyn_payload);
WPT_STATIC_PAYLOAD_DEF(m_static_payload);
WPT_PTU_STATIC_PAYLOAD_DEF(m_ptu_static_payload);
WPT_ALERT_PAYLOAD_DEF(m_alert_payload);
WPT_CONTROL_PAYLOAD_DEF(m_control_payload);

//TIMERS FUNCTION

void dynamic_param_timeout_handler(void *arg);


//* BLE FUNCTIONS

int gatt_svr_init(void);

/* update the alert parameter */
int gatt_svr_chr_notify_pru_alert(struct os_mbuf* om, uint16_t conn_handle);


//*PAYLOAD FUNCTION DEFINITION

//esp_gatt_status_t wpt_control_param_unpack(ble_wpt_t *p_wpt, wpt_control_payload_t *control_payload);


#endif
