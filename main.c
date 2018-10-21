/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_drv_twi.h"
#include "SEGGER_RTT.h"
#include "math.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"



#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "DECA_LB"                               		/**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */



// IMU 
//Registers
const uint8_t CHANNEL_COMMAND = 0;
const uint8_t CHANNEL_EXECUTABLE = 1;
const uint8_t CHANNEL_CONTROL = 2;
const uint8_t CHANNEL_REPORTS = 3;
const uint8_t CHANNEL_WAKE_REPORTS = 4;
const uint8_t CHANNEL_GYRO = 5;

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

//Global Variables
uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
uint8_t shtpData[MAX_PACKET_SIZE];
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
uint8_t commandSequenceNumber = 0; //Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
uint32_t metaData[MAX_METADATA_SIZE]; //There is more than 10 words in a metadata record but

#define BNO_ADDRESS               	0x4B            // Device address when SA0 Pin 17 = GND; 0x4B SA0 Pin 17 = VDD
#define QP(n)                       (1.0f / (1 << n))                   // 1 << n ==  2^-n
#define radtodeg                    (180.0f / (22/7))


/* TWI instance. */
//static const nrf_drv_twi_t m_twi_mma_7660 = NRF_DRV_TWI_INSTANCE(0);
static const nrf_drv_twi_t m_twi_bno = NRF_DRV_TWI_INSTANCE(0);

static bool initialized = false;                  					// check if IMU has been initialized
const uint8_t quat_report = 0x05;          									// defines kind of rotation vector (0x05), geomagnetic (0x09), AR/VR (0x28),
const int reporting_frequency    = 400;           					// reporting frequency in Hz  // note that serial output strongly reduces data rate
const uint8_t B0_rate = 1000000 / reporting_frequency;      // calculate LSB (byte 0)
const uint8_t B1_rate = B0_rate >> 8;                       // calculate byte 1
static uint8_t readSuccess = 0;                             // confirms quaternion has been read successfully
static float q0,q1,q2,q3;                                		// quaternions q0 = qw 1 = i; 2 = j; 3 = k;
static float h_est;                                      		// heading accurracy estimation
static uint8_t stat_;                                    		// Status (0-3)
static uint8_t cargo[23] = {0}; 														// holds in coming data

static float quatI = 0;
static float quatJ = 0;
static float quatK = 0;
static float quatReal = 0;
static float quatI_Prev = 0;
static float quatJ_Prev = 0;
static float quatK_Prev = 0;
static float quatReal_Prev = 0;

static bool reset = false;																	
static bool requestID = false;
uint8_t i2C_event = 0;
uint8_t str[24]; 																						// Holds the string from the IMU

// function prototypes
static void set_feature_cmd_QUAT(); 												// configure quaternion output
float qToFloat_(int16_t, uint8_t);    											// convert q point data into float
static void sendPacket_IMU(uint8_t, uint8_t);               // send data to IMU
void resetIMU();                                            // reset the IMU
void requestProductID();                                    // request the product ID
bool sendDataPacket(uint8_t, uint8_t);											// send data packet to IMU
bool setFeature(uint8_t, uint16_t, uint32_t);								// set a feature on the IMU

bool TX_Complete = false;
const bool UART_EN = false;
static uint8_t sampleTimes = 0;

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for sending data to connected client
 *
 * @details This function is called from the sendSettings and sendbat functions that are
 *          are used to send the device settings and battery level to the connected client
 *          had to edit ble_nus_string_send function in the ble_nus.c file and commented out
 *          the IF statement that checks if notifications are enabled (they are enabled)
 *
 * @param[in] array[]  data to be sent.
 * @param[in] arraySize  length of data that is being sent.
 */
static void sendData(uint8_t array[], uint8_t arraySize){
		uint32_t       err_code;
		uint8_t buff1[20] = {0};
		uint8_t buff2[4] = {0, 0, 0, 0};
		
		if(str[20] == 0 && str[21] == 0 && str[22] == 0 && str[23] == 0) err_code = ble_nus_string_send(&m_nus, array, BLE_NUS_MAX_DATA_LEN);
		else
		{
				
				err_code = ble_nus_string_send(&m_nus, array, BLE_NUS_MAX_DATA_LEN);
				nrf_delay_ms(5);
				for(uint8_t i = 0; i < 4; i++)
				{
						buff2[i] = array[20 + i];
						if((char)array[20 + i] == ';')
						{
								err_code = ble_nus_string_send(&m_nus, buff2, i + 1);
								break;
						}
				}
				
		}
		////SEGGER_RTT_printf(0,"DS\n");
		//err_code != BLE_ERROR_NO_TX_BUFFERS
		if (err_code != NRF_SUCCESS &&
        err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
        err_code != NRF_ERROR_INVALID_STATE
        )
    {
				////SEGGER_RTT_printf(0, "Error: %d", err_code);
        //APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
						
						////SEGGER_RTT_printf(0,"\r\nConnected!\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						TX_Complete = true;
						if(UART_EN)
						{
								printf("*");
								//SEGGER_RTT_printf(0,"\r\nUART requested!\r\n");
						}
						else
						{	
								//SEGGER_RTT_printf(0,"\r\nInitial data submission!\r\n");
								//sendData(str, sizeof(str));
						}
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
						//SEGGER_RTT_printf(0,"\r\nDisconnected!\r\n");
						TX_Complete = false;
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
				case BLE_EVT_TX_COMPLETE:
						//sendData(str, sizeof(str));
						//TX_Complete = true;
						////SEGGER_RTT_printf(0,"\n\rData sent\r\n");
						break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice 
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
        
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                if(TX_Complete)
								{
										err_code = ble_nus_string_send(&m_nus, data_array, index);
										////SEGGER_RTT_printf(0,"Data sent!\r\n");
										TX_Complete =  false;
								}
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        6,//RX_PIN_NUMBER,
        5,//TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */


/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
    ret_code_t err_code;
    //static sample_t m_sample;
    readSuccess = 0;                           // nothing read
		int s = 0;
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:				
						if((p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)) // receive event
						{ 
								if(!initialized)
								{
										initialized = true;  
										return;
								}
								else 
									
								err_code = nrf_drv_twi_rx(&m_twi_bno, BNO_ADDRESS, (uint8_t*)&cargo, sizeof(cargo));
								APP_ERROR_CHECK(err_code);	
						}
						else{
								if(!initialized)
								{
										initialized = true;  
										return;
								}
								readSuccess = true;
								/*//SEGGER_RTT_printf(0, "EVTTX\r\n");
								if(sampleTimes == 0)
								{
										readSuccess = true;
								}
								else
								{
										err_code = nrf_drv_twi_tx(&m_twi_bno, BNO_ADDRESS, (uint8_t*)&cargo, sizeof(cargo));
										sampleTimes++;
										if(sampleTimes == 3)sampleTimes = 0;
								}*/
						}
            break;
        default:
            break;        
    }   
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_bno_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&m_twi_bno, &twi_bno_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_twi_bno);
}

/**
 * @brief BNO IMU initialization.
 */
void initializeIMU(){
		ret_code_t err_code;
		uint8_t reg[2] = {0, 0};
		
		// check if i2C communication is sucessful
		err_code = nrf_drv_twi_tx(&m_twi_bno, BNO_ADDRESS, reg, sizeof(reg), false);  
		nrf_delay_ms(100);		
    while (!initialized){	
      ////SEGGER_RTT_printf(0,".");
    }
		//SEGGER_RTT_printf(0,"\nIMU Available!\r\n");
		nrf_delay_ms(100);
		if(setFeature(SENSOR_REPORTID_ROTATION_VECTOR, 1, 0))
		{
				//SEGGER_RTT_printf(0,"\nQuats set\r\n");
		}
		nrf_delay_ms(100);
}

static void get_QUAT()
{  
    readSuccess = 0;       
		ret_code_t err_code;	
		uint8_t reg[4] = {0, 0};
		
		err_code = nrf_drv_twi_rx(&m_twi_bno, BNO_ADDRESS, (uint8_t*)&cargo, sizeof(cargo));
		if(sampleTimes == 0)
		{
				nrf_delay_ms(10);
				//APP_ERROR_CHECK(err_code);
				while(!readSuccess){
						////SEGGER_RTT_printf(0, ".");
				}
		}

    //Check to see if this packet is a sensor reporting its data to us
    if((readSuccess == 1) && (cargo[9] == quat_report) && (cargo[2] == 0x03) && (cargo[4] == 0xFB)){    //  && ((cargo[10]) == next_data_seqNum ) check for report and incrementing data seqNum
				//next_data_seqNum = ++cargo[10];                                         // predict next data seqNum              
        stat_ = cargo[11] & 0x03;                                                 // bits 1:0 contain the status (0,1,2,3)  
				//for(uint8_t i = 0; i < 8; i++)//SEGGER_RTT_printf(0,"%d, ", cargo[13 + i]);
        ////SEGGER_RTT_printf(0,"\r\n");
		
				float qI = (((int16_t)cargo[14] << 8) | cargo[13] ); 
        float qJ = (((int16_t)cargo[16] << 8) | cargo[15] );
        float qK = (((int16_t)cargo[18] << 8) | cargo[17] );
        float qReal = (((int16_t)cargo[20] << 8) | cargo[19] ); 
		
				/*//SEGGER_RTT_printf(0,"%d, ", qI);
				//SEGGER_RTT_printf(0,"%d, ", qJ);
				//SEGGER_RTT_printf(0,"%d, ", qK);
				//SEGGER_RTT_printf(0,"%d\r\n", qReal);*/
		
        quatReal = qToFloat_(qReal, 14); //pow(2, 14 * -1);//QP(14); 
        quatI = qToFloat_(qI, 14); //pow(2, 14 * -1);//QP(14); 
        quatJ = qToFloat_(qJ, 14); //pow(2, 14 * -1);//QP(14); 
        quatK = qToFloat_(qK, 14); //pow(2, 14 * -1);//QP(14);                  // apply Q point (quats are already unity vector)

        //if (quat_report == 0x05){  // heading accurracy only in some reports available
        h_est = (((int16_t)cargo[22] << 8) | cargo[21] );                        // heading accurracy estimation  
        h_est *= QP(12);                                                         // apply Q point 
        h_est *= radtodeg;                                                       // convert to degrees                
        //}
    }
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float qToFloat_(int16_t fixedPointValue, uint8_t qPoint)
{
		float qFloat = fixedPointValue;
		qFloat *= pow(2, (qPoint * -1));
		return (qFloat);
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
bool setFeature(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
		long microsBetweenReports = (long)timeBetweenReports * 1000L;
		
		shtpData[4] = SHTP_REPORT_SET_FEATURE_COMMAND; //Set feature command. Reference page 55
		shtpData[5] = reportID; //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
		shtpData[6] = 0; //Feature flags
		shtpData[7] = 0; //Change sensitivity (LSB)
		shtpData[8] = 0; //Change sensitivity (MSB)
		shtpData[9] = (microsBetweenReports >> 0) & 0xFF; //Report interval (LSB) in microseconds. 0x7A120 = 500ms
		shtpData[10] = (microsBetweenReports >> 8) & 0xFF; //Report interval
		shtpData[11] = (microsBetweenReports >> 16) & 0xFF; //Report interval
		shtpData[12] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
		shtpData[13] = 0; //Batch Interval (LSB)
		shtpData[14] = 0; //Batch Interval
		shtpData[15] = 0; //Batch Interval
		shtpData[16] = 0; //Batch Interval (MSB)
		shtpData[17] = (specificConfig >> 0) & 0xFF; //Sensor-specific config (LSB)
		shtpData[18] = (specificConfig >> 8) & 0xFF; //Sensor-specific config
		shtpData[19] = (specificConfig >> 16) & 0xFF; //Sensor-specific config
		shtpData[20] = (specificConfig >> 24) & 0xFF; //Sensor-specific config (MSB)

		//Transmit packet on channel 2, 17 bytes
		return sendDataPacket(CHANNEL_CONTROL, 17);
}

bool sendDataPacket(uint8_t channelNumber, uint8_t dataLength)
{
    uint8_t packetLength = dataLength + 4; //Add four bytes for the header
		ret_code_t err_code;
    //if(packetLength > I2C_BUFFER_LENGTH) return(false); //You are trying to send too much. Break into smaller packets.

    //set the 4 byte packet header
    shtpData[0] = packetLength & 0xFF; //Set feature command. Reference page 55
		shtpData[1] = packetLength >> 8; //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
		shtpData[2] = channelNumber; //Feature flags
		shtpData[3] = sequenceNumber[channelNumber]++; //Change sensitivity (LSB)
    
		err_code = nrf_drv_twi_tx(&m_twi_bno, BNO_ADDRESS, shtpData, sizeof(shtpData), false);  
		nrf_delay_ms(10);
	
    if (err_code != 0)
    {
      return (false);
    }
    return true;
}
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
		
		// Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
		uart_init();
    //buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();	
		
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
		
		if(UART_EN == true)
		{
				uart_init();
				//SEGGER_RTT_printf(0,"\r\nUART Initialized!\r\n");
		}
		else
		{		
				nrf_delay_ms(1000);
				SEGGER_RTT_printf(0,"\n\rTWI sensor example\r\n");
				twi_init();
				SEGGER_RTT_printf(0,"\n\rTWI Initialized\r\n");
				initializeIMU();
				SEGGER_RTT_printf(0,"\n\rIMU Initialized\r\n");
		}
    
		// Enter main loop.
    for (;;)
    {
				get_QUAT();
				if(sampleTimes == 0)
				{
						memset(str, 0, sizeof(str));
						sprintf((char*)&str[0], "%3.2f,%3.2f,%3.2f,%3.2f;", quatReal, quatI, quatJ, quatK);
						if(TX_Complete)sendData(str, sizeof(str));
						sampleTimes++;
						//SEGGER_RTT_WriteString(0,"Sent NUS \r\n");
				}
				sampleTimes++;
				if(sampleTimes == 3)sampleTimes = 0;
				//nrf_delay_ms(100);
        power_manage();
    }
}


/** 
 * @}
 */
