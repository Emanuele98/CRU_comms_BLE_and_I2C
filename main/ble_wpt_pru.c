#include "include/ble_wpt_pru.h"

/**********************************************************************************/
/**                            Restricted globals                                **/
/**********************************************************************************/

/* Defines what type of task the PRU will run on */
#define PRU_TASKS_CORE        1
#define PRU_TASK_STACK_SIZE   2000
#define PRU_TASK_PRIORITY     16

/**********************************************************************************/
/**                       Static function declarations                           **/
/**********************************************************************************/

/* Debug tag */
static const char* TAG = "BLE_PHERIPERAL";

//*declare service
static const ble_uuid128_t wpt_service_uuid = BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0xFE, 0xFF, 0x55, 0x64);

//*declare characteristics
static const ble_uuid128_t WPT_PRU_STAT_UUID =
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x73, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_PTU_STAT_UUID =
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x71, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_PRU_DYN_UUID = 
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x74, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_CONTROL_UUID =
    BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x70, 0xE6, 0x55, 0x64);

static const ble_uuid128_t WPT_ALERT_UUID =
 BLE_UUID128_INIT(0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x72, 0xE6, 0x55, 0x64);


//*CHR CALLBACK FUNCTIONS
static int gatt_svr_chr_read_pru_static(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

static int gatt_svr_chr_write_ptu_static(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);
    
static int gatt_svr_chr_read_pru_dynamic(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

static int gatt_svr_chr_write_pru_control(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

static int gatt_svr_chr_notify_alert_dsc(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

//static uint16_t alert_handle;


static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &wpt_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {   {
                // Characteristic: pru static payload.
                .uuid = &WPT_PRU_STAT_UUID.u,
                .access_cb = gatt_svr_chr_read_pru_static,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                // Characteristic: ptu static payload.
                .uuid = &WPT_PTU_STAT_UUID.u,
                .access_cb = gatt_svr_chr_write_ptu_static,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                // Characteristic: pru dynamic payload.
                .uuid = &WPT_PRU_DYN_UUID.u,
                .access_cb = gatt_svr_chr_read_pru_dynamic,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                // Characteristic: pru control payload.
                .uuid = &WPT_CONTROL_UUID.u,
                .access_cb = gatt_svr_chr_write_pru_control,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                // Characteristic: pru alert payload.
                .uuid = &WPT_ALERT_UUID.u,
                .access_cb = gatt_svr_chr_notify_alert_dsc,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0, // No more characteristics in this service.
            }
        },
    },

    {
        0, // No more services. 
    },
};

static int
gatt_svr_chr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                   void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
    ESP_LOGW(TAG, "err! length=%d", om_len);

        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        ESP_LOGW(TAG, "oh shit");
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

static int gatt_svr_chr_write_ptu_static(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    //ESP_LOGE(TAG, "WRITE PTU STATIC CALLBACK");

    int err_code ;

    // uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om); // 17! nice

    uint8_t data[PTU_STATIC_CHAR_SIZE];

    err_code = gatt_svr_chr_write(ctxt->om,
                            sizeof data,
                            sizeof data,
                            &data, NULL);
    
    m_ptu_static_payload.optional_fields = data[0];
    m_ptu_static_payload.ptu_power = data[1];
	m_ptu_static_payload.max_impedance = data[2];
	m_ptu_static_payload.max_load = data[3];
	m_ptu_static_payload.RFU1 = ((uint16_t)data[4]<<8)|data[5];
	m_ptu_static_payload.ptu_class = data[6];
	m_ptu_static_payload.hard_rev = data[7];
	m_ptu_static_payload.firm_rev = data[8];
	m_ptu_static_payload.protocol_rev = data[9];
	m_ptu_static_payload.max_num_devices = data[10];
	m_ptu_static_payload.company_id = (((uint16_t)data[11])<<8)|data[12];
	m_ptu_static_payload.RFU2 = (((uint32_t)data[13])<<24)|
							(((uint32_t)data[13])<<16)|
							(((uint32_t)data[13])<<8)|
							data[13];  // 17bites of data


    //ESP_LOGW(TAG, "value [0]=%d", m_ptu_static_payload.optional_fields);

    //Start app timers
    xTimerStart(dynamic_t_handle, 0);
    xTimerStart(alert_t_handle, 0);

    return err_code;

}


static int gatt_svr_chr_read_pru_static(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    //ESP_LOGE(TAG, "READ PRU STATIC CALLBACK");

    int err_code ;

    //INIT STATIC PAYLOAD 
    m_static_payload.optional_fields = OPTIONAL_FIELDS_STAT;
	m_static_payload.protocol_rev = PROTOCOL_REVISION;
	m_static_payload.pru_cat = PRU_CATEGORY;
	m_static_payload.pru_info = PRU_INFORMATION;
	m_static_payload.hard_rev = PRU_HARD_REVISION;
	m_static_payload.firm_rev = PRU_FIRM_REVISION;
	m_static_payload.prect_max = PRECT_MAXIMUM;
	m_static_payload.company_id = COMPANY_ID;
	
	// Setting static values for Vmin, Vset and Vhigh 
	m_static_payload.vrect_min_stat = VRECT_DYN_MINIMUM;
	m_static_payload.vrect_high_stat = VRECT_DYN_HIGH;
	m_static_payload.vrect_set = VRECT_DYN_SET;

    /* Setting dynamic values for Vmin, Vset and Vhigh */
    m_dyn_payload.vrect_min_dyn = VRECT_DYN_MINIMUM;
	m_dyn_payload.vrect_high_dyn = VRECT_DYN_HIGH;
	m_dyn_payload.vrect_set_dyn = VRECT_DYN_SET;

    unsigned char *rfu_p = (unsigned char*)&m_static_payload.RFU2;

    uint8_t data[PRU_STATIC_CHAR_SIZE] = {m_static_payload.optional_fields,
    											m_static_payload.protocol_rev,
                                                m_static_payload.RFU1,
											    m_static_payload.pru_cat,
											    m_static_payload.pru_info,
											    m_static_payload.hard_rev,
											    m_static_payload.firm_rev,
											    m_static_payload.prect_max,
											    m_static_payload.vrect_min_stat >> 8,
											    m_static_payload.vrect_min_stat & 0x00FF,
											    m_static_payload.vrect_high_stat >> 8,
											    m_static_payload.vrect_high_stat & 0x00FF,
											    m_static_payload.vrect_set >> 8,
											    m_static_payload.vrect_set & 0x00FF,
											    m_static_payload.company_id >> 8,
											    m_static_payload.company_id & 0x00FF,
											    rfu_p[0],rfu_p[1],rfu_p[2],rfu_p[3]}; // 20 bytes of data
                                                
    err_code = os_mbuf_append(ctxt->om, &data,
                        sizeof data);
    return err_code;
}

static int gatt_svr_chr_read_pru_dynamic(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    //ESP_LOGE(TAG, "READ PRU DYNAMIC CALLBACK");

    int err_code;

	uint8_t data[PRU_DYNAMIC_CHAR_SIZE] = {m_dyn_payload.optional_fields,
										    m_dyn_payload.vrect >> 8,
											m_dyn_payload.vrect & 0x00FF,
											m_dyn_payload.irect >> 8,
											m_dyn_payload.irect & 0x00FF,
											m_dyn_payload.vout >> 8,
											m_dyn_payload.vout & 0x00FF,
											m_dyn_payload.iout >> 8,
											m_dyn_payload.iout & 0x00FF,
											m_dyn_payload.temp_ratio,
											m_dyn_payload.vrect_min_dyn >> 8,
											m_dyn_payload.vrect_min_dyn & 0x00FF,
											m_dyn_payload.vrect_set_dyn >> 8,
											m_dyn_payload.vrect_set_dyn & 0x00FF,
											m_dyn_payload.vrect_high_dyn >> 8,
											m_dyn_payload.vrect_high_dyn & 0x00FF,
											m_dyn_payload.pru_alert,
											m_dyn_payload.tester_command,
											m_dyn_payload.dynamic_pru_info,
											m_dyn_payload.RFU}; // 20 bytes of data


    //ESP_LOGW(TAG, "- DYN CHR - rx voltage = %d", m_dyn_payload.vrect);
    //ESP_LOGW(TAG, "- DYN CHR - rx voltage = %d", m_dyn_payload.irect);
    //ESP_LOGW(TAG, "- DYN CHR - rx temperature = %d", m_dyn_payload.temp_ratio);

    err_code = os_mbuf_append(ctxt->om, &data,
                        sizeof data);
    return err_code;
}

static int gatt_svr_chr_write_pru_control(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{

    //ESP_LOGE(TAG, "WRITE PRU CONTROL CALLBACK");

    int err_code = 0 ;

    //uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om); // 5

    uint8_t data[PRU_CONTROL_CHAR_SIZE];

    err_code = gatt_svr_chr_write(ctxt->om,
                            sizeof data,
                            sizeof data,
                            &data, NULL);
    
    m_control_payload.enables = data[0];
	m_control_payload.permission = data[1];
	m_control_payload.time_set = data[2];
	m_control_payload.RFU = ((uint16_t)data[3]<<8)|data[4];

    return err_code;

}

static int gatt_svr_chr_notify_alert_dsc(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    //ESP_LOGE(TAG, "WRITE ALERT CALLBACK");
    int err_code = 0 ;

    uint8_t data[PRU_ALERT_CHAR_SIZE];

    //simulate alert situation
    //m_alert_payload.alert_field.overtemperature = 1;

    data[0] = m_alert_payload.alert_field.internal;

    // read alert chr
    err_code = os_mbuf_append(ctxt->om, &data,
                        sizeof data);
    if (err_code != ESP_OK)
    {
        return err_code;
    }

    //NOTIFY alert chr
    struct os_mbuf *om;
    om = ble_hs_mbuf_from_flat(data, sizeof(data));

    err_code = ble_gattc_notify_custom(conn_handle, attr_handle, om);


    return err_code;
}



int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }
    

    return 0;
}
