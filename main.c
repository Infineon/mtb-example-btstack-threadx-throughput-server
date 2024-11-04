/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the Bluetooth LE Threadx Throughput Server
* Example for ModusToolbox.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
*        Header Files
*******************************************************************************/

/* Header file includes */
#include <string.h>
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "GeneratedSource/cycfg_gatt_db.h"
#include "GeneratedSource/cycfg_bt_settings.h"
#include "app_bt_utils.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_memory.h"
#include "wiced_bt_stack.h"
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "cyhal_gpio.h"
#include "wiced_bt_l2c.h"
#include "cyabs_rtos.h"
#include "stdlib.h"
#include <inttypes.h>
#include "app_bt_utils.h"

#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif


/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
/**
 * @brief Typdef for function used to free allocated buffer to stack
 */
typedef void (*pfn_free_buffer_t)(uint8_t *);


#define NOTIFY_TASK_NAME           "Notify Task"
#define TPUT_TASK_NAME              "Tput Task"
#define TASK_STACK_SIZE              (8192)
#define TASK_PRIORITY        (CY_RTOS_PRIORITY_NORMAL)

#define NOTIFICATION_DATA_SIZE           (244)

/* Connection update parameters */
#define CONNECTION_INTERVAL               		(28)		/* (1.25 * CONNECTION_INTERVAL)ms */
#define SUPERVISION_TIMEOUT             		(1000)
#define CONN_INTERVAL_MULTIPLIER				(1.25f)
#define TPUT_TIMER_UPDATE   					(5*3000000)
#define TPUT_FREQUENCY 							(3000000)
#define PACKET_PER_EVENT						(10)
/**
 * @brief This enumeration combines the advertising, connection states from two
 *        different callbacks to maintain the status in a single state variable
 */
typedef enum
{
    APP_BT_ADV_OFF_CONN_OFF,
    APP_BT_ADV_ON_CONN_OFF,
    APP_BT_ADV_OFF_CONN_ON
} app_bt_adv_conn_mode_t;

typedef struct
{
    wiced_bt_device_address_t             remote_addr;   /* remote peer device address */
    uint16_t                              conn_id;       /* connection ID referenced by the stack */
    uint16_t                              mtu;           /* MTU exchanged after connection */
    double                                conn_interval; /* connection interval negotiated */
    wiced_bt_ble_host_phy_preferences_t   rx_phy;        /* RX PHY selected */
    wiced_bt_ble_host_phy_preferences_t   tx_phy;        /* TX PHY selected */

} conn_state_info_t;

/*******************************************************************************
*        Variable Definitions
*******************************************************************************/

/* Variables to hold GATT notification bytes sent and GATT Write bytes received
 * successfully */
static unsigned long gatt_notif_tx_bytes = 0u;
static unsigned long gatt_write_rx_bytes = 0u;

/**
 * @brief Variable to store handle of tasks created to update the throughput and send
          notifications
 */
static cy_thread_t notify_task_pointer, tput_task_pointer;

/**
 * @brief variable to track connection and advertising state
 */
static app_bt_adv_conn_mode_t app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;

static uint64_t notify_task_stack[TASK_STACK_SIZE], tput_task_stack[TASK_STACK_SIZE];

uint8_t notification_data_seq[NOTIFICATION_DATA_SIZE];

cy_semaphore_t semaphore, congestion;

uint8_t tput_fun = 1;

/* Variable to store connection state information*/
static conn_state_info_t conn_state_info;

/**
 * @brief Variable for 5 sec timer object
 */
static cyhal_timer_t tput_timer_obj;

/**
 * @brief Configure timer for 5 sec
 */
const cyhal_timer_cfg_t tput_timer_cfg =
    {
        .compare_value = 0,                    /* Timer compare value, not used */
        .period = TPUT_TIMER_UPDATE,            /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,       /* Timer counts up */
        .is_compare = false,                   /* Don't use compare mode */
        .is_continuous = true,                 /* Run timer indefinitely */
        .value = 0                             /* Initial value of counter */
};

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/

/* GATT Event Callback Functions */

static wiced_bt_gatt_status_t app_bt_gatt_req_read_handler          (uint16_t conn_id,
                                                                     wiced_bt_gatt_opcode_t opcode,
                                                                     wiced_bt_gatt_read_t *p_read_req,
                                                                     uint16_t len_requested);
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
                                                                   wiced_bt_gatt_opcode_t opcode,
                                                                   wiced_bt_gatt_read_by_type_t *p_read_req,
                                                                   uint16_t len_requested);

static wiced_bt_gatt_status_t app_bt_connect_event_handler          (wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t app_bt_server_event_handler           (wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t app_bt_gatt_event_callback            (wiced_bt_gatt_evt_t event,
                                                                     wiced_bt_gatt_event_data_t *p_event_data);
static wiced_bt_gatt_status_t app_bt_write_handler                  (wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t app_bt_set_value                      (uint16_t attr_handle, uint8_t *p_val, uint16_t len);

/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t  app_bt_management_callback            (wiced_bt_management_evt_t event,
                                                                     wiced_bt_management_evt_data_t *p_event_data);
static void                   app_bt_init                           (void);

/* Task to send notifications */
void notify_task(cy_thread_arg_t arg);

/* Task to calculate throughput every 5 seconds */
void tput_task(cy_thread_arg_t arg);

/* HAL timer callback registered when timer reaches terminal count */
void tput_timer_callb(void *callback_arg, cyhal_timer_event_t event);


/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

 /**
 * Function Name: hci_trace_cback
 *
 * Function Description:
 *   @brief This callback routes HCI packets to debug uart.
 *
 *   @param wiced_bt_hci_trace_type_t type : HCI trace type
 *   @param uint16_t length : length of p_data
 *   @param uint8_t* p_data : pointer to data
 *
 *   @return None
 *
 */
#ifdef ENABLE_BT_SPY_LOG
void hci_trace_cback(wiced_bt_hci_trace_type_t type,
                     uint16_t length, uint8_t* p_data)
{
    cybt_debug_uart_send_hci_trace(type, length, p_data);
}
#endif

/**
 * Function Name:
 * app_bt_alloc_buffer
 *
 * Function Description:
 * @brief  This Function allocates the buffer of requested length
 *
 * @param len            Length of the buffer
 *
 * @return uint8_t*      pointer to allocated buffer
 */
static uint8_t *app_bt_alloc_buffer(uint16_t len)
{
    uint8_t *p = (uint8_t *)malloc(len);
    printf( "%s() len %d alloc %p \n", __FUNCTION__,len, p);
    return p;
}

/**
 * Function Name:
 * app_bt_free_buffer
 *
 * Function Description:
 * @brief  This Function frees the buffer requested
 *
 * @param p_data         pointer to the buffer to be freed
 *
 * @return void
 */
static void app_bt_free_buffer(uint8_t *p_data)
{
    if (p_data != NULL)
    {
        printf( "%s()        free:%p \n",__FUNCTION__, p_data);
        free(p_data);
    }
}

/**
 * Function Name:
 * main
 *
 * Function Description :
 *  @brief Entry point to the application. Set device configuration and start BT
 *         stack initialization.  The actual application initialization will happen
 *         when stack reports that BT device is ready.
 */
int main()
{
    cy_rslt_t cy_result;
    wiced_result_t  result;

    /* Initialize the board support package */
    cy_result = cybsp_init();
    if (CY_RSLT_SUCCESS != cy_result)
    {
        printf("BSP init failed \n");
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

#ifdef ENABLE_BT_SPY_LOG
{
    cybt_debug_uart_config_t config = {
        .uart_tx_pin = CYBSP_DEBUG_UART_TX,
        .uart_rx_pin = CYBSP_DEBUG_UART_RX,
        .uart_cts_pin = CYBSP_DEBUG_UART_CTS,
        .uart_rts_pin = CYBSP_DEBUG_UART_RTS,
        .baud_rate = DEBUG_UART_BAUDRATE,
        .flow_control = TRUE};
    cybt_debug_uart_init(&config, NULL);
}
#else
{

    /* Initialize retarget-io to use the debug UART port */
    cy_result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
}
#endif //ENABLE_BT_SPY_LOG

    printf("\n\n**** BLE Throughput: GATT Server Application Start ****\n\n");

    /* retarget-io init failed. Stop program execution */
    if (cy_result != CY_RSLT_SUCCESS)
    {
         printf("Retarget IO init failed\n");
         CY_ASSERT(0);
    }

    /* Register call back and configuration with stack */
    result = wiced_bt_stack_init(app_bt_management_callback, &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if (WICED_BT_SUCCESS != result)
    {
        printf( "Bluetooth Stack Initialization failed %x !! \n", result);
        CY_ASSERT(0);
    }

    /*Create Notify task*/
    result = cy_rtos_thread_create(&notify_task_pointer,
                                   &notify_task,
                                   NOTIFY_TASK_NAME,
                                   &notify_task_stack,
                                   TASK_STACK_SIZE,
                                   CY_RTOS_PRIORITY_NORMAL,
                                   0);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Notify task creation failed 0x%X\n", result);
    }

    result = cy_rtos_thread_create(&tput_task_pointer,
                                   &tput_task,
                                   TPUT_TASK_NAME,
                                   &tput_task_stack,
                                   TASK_STACK_SIZE,
                                   CY_RTOS_PRIORITY_NORMAL,
                                   0);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Tput task creation failed 0x%X\n", result);
    }

    result = cy_rtos_semaphore_init(&semaphore, 1, 0);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Semaphore initialization failed 0x%X\n", result);
    }


    result = cy_rtos_semaphore_init(&congestion, 1, 0);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Congestion semaphore initialization failed 0x%X\n", result);
    }
}

/**
* Function Name: app_bt_management_callback
*
* Function Description:
* @brief
*  This is a Bluetooth stack event handler function to receive management events
*  from the Bluetooth stack and process as per the application.
*
* @param wiced_bt_management_evt_t       Bluetooth LE event code of one byte length
* @param wiced_bt_management_evt_data_t  Pointer to Bluetooth LE management event
*                                        structures
*
* @return wiced_result_t Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
*/
wiced_result_t app_bt_management_callback(wiced_bt_management_evt_t event,
                                          wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result  = WICED_BT_ERROR;
    wiced_bt_device_address_t bda = {0};
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;
    wiced_bool_t conn_param_status = 0;


    switch (event)
    {
    case BTM_ENABLED_EVT:

        /* Bluetooth Controller and Host Stack Enabled */
        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            /* Initialize the application */
            wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address, BLE_ADDR_PUBLIC);
            /* Bluetooth is enabled */
            wiced_bt_dev_read_local_addr(bda);
            printf("Local Bluetooth Address: ");
            /* Perform application-specific initialization */
            app_bt_init();
            result = WICED_BT_SUCCESS;
        }
        else
        {
            printf( "Failed to initialize Bluetooth controller and stack 0x%X\n", p_event_data->enabled.status);
        }

        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:        /* Local identity Keys Update */
        result = WICED_BT_SUCCESS;
        break;

    case BTM_BLE_PHY_UPDATE_EVT:
    {
    	conn_state_info.rx_phy = p_event_data->ble_phy_update_event.rx_phy;
    	conn_state_info.tx_phy = p_event_data->ble_phy_update_event.tx_phy;
    	printf("Selected RX PHY - %dM\nSelected TX PHY - %dM\n", conn_state_info.rx_phy,conn_state_info.tx_phy);
        print_bd_address(conn_state_info.remote_addr);
        conn_param_status = wiced_bt_l2cap_update_ble_conn_params(conn_state_info.remote_addr,
                    		CONNECTION_INTERVAL,CONNECTION_INTERVAL + 1,
        					CY_BT_CONN_LATENCY,SUPERVISION_TIMEOUT);
        /* Send connection parameter update request to peripheral */
        if(!conn_param_status)
        {
            printf("Failed to Send Connection update parameter request \r\n");
        }
        result = WICED_BT_SUCCESS;
        break;
    }

    case BTM_BLE_CONNECTION_PARAM_UPDATE:
        /* Connection parameters updated */
        printf( "BTM_BLE_CONNECTION_PARAM_UPDATE \r\n");
        printf( "ble_connection_param_update.bd_addr: ");
        print_bd_address(p_event_data->ble_connection_param_update.bd_addr);
        printf( "ble_connection_param_update.conn_interval       : %f ms\r\n",(double)(p_event_data->ble_connection_param_update.conn_interval*CONN_INTERVAL_MULTIPLIER));
        printf( "ble_connection_param_update.conn_latency        : %d\r\n",p_event_data->ble_connection_param_update.conn_latency);
        printf( "ble_connection_param_update.supervision_timeout : %d\r\n",p_event_data->ble_connection_param_update.supervision_timeout);
        printf( "ble_connection_param_update.status              : 0x%x\r\n\n",p_event_data->ble_connection_param_update.status);
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

        /* Advertisement State Changed */
        p_adv_mode = &p_event_data->ble_advert_state_changed;

        if (BTM_BLE_ADVERT_OFF == *p_adv_mode)
        {
            /* Advertisement stopped */
            printf("Advertisement Stopped\n");

            /* Check connection status after advertisement stops */
            if (0 == conn_state_info.conn_id)
            {
                app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_OFF;
            }
            else
            {
                app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
            }
        }
        else
        {
            /* Advertisement Started */
            printf("Advertisement Started\n");
            app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;
        }

        result = WICED_BT_SUCCESS;
        break;

    default:

        result = WICED_BT_SUCCESS;
        break;
    }

    return result;
}

/**
 *  Function Name:
 *  app_bt_init
 *
 *  Function Description:
 *  @brief  This function handles application level initialization tasks and is
 *          called from the BT management callback once the Bluetooth LE stack enabled event
 *          (BTM_ENABLED_EVT) is triggered This function is executed in the BTM_ENABLED_EVT
 *          management callback.
 *
 *  @param  void
 *
 *  @return wiced_result_t WICED_SUCCESS or WICED_failure
 */
static void app_bt_init(void)
{
    cy_rslt_t cy_result = CY_RSLT_SUCCESS;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    wiced_result_t result;
    
#ifdef ENABLE_BT_SPY_LOG
    wiced_bt_dev_register_hci_trace(hci_trace_cback);
#endif

    printf("**Discover device with \"TPUT\" name*\n");

    /* Initialize the data packet to be sent as GATT notification to the peer device */
    for(uint8_t iterator = 0; iterator < NOTIFICATION_DATA_SIZE; iterator++)
    {
        notification_data_seq[iterator] = iterator;
    }

    /* Initialize the HAL timer used to count seconds */
    cy_result = cyhal_timer_init(&tput_timer_obj, NC, NULL);
    if (CY_RSLT_SUCCESS != cy_result)
    {
        printf("Throughput timer init failed !\n");
    }
    /* Configure the timer for 5 seconds */
    cyhal_timer_configure(&tput_timer_obj, &tput_timer_cfg);
    cy_result = cyhal_timer_set_frequency(&tput_timer_obj, TPUT_FREQUENCY);
    if (CY_RSLT_SUCCESS != cy_result)
    {
        printf("Throughput timer set frequency failed !\n");
    }
    /* Register for a callback whenever timer reaches terminal count */
    cyhal_timer_register_callback(&tput_timer_obj, tput_timer_callb, NULL);
    cyhal_timer_enable_event(&tput_timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3, true);

    /* Disable pairing for this application */
    wiced_bt_set_pairable_mode(WICED_FALSE, 0);

    /* Set Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
                                            cy_bt_adv_packet_data);

    /* Register with BT stack to receive GATT callback */
    status = wiced_bt_gatt_register(app_bt_gatt_event_callback);
    if (WICED_BT_GATT_SUCCESS != status)
    {
        printf("GATT Registeration failed  because of error: %d \n", status);
        CY_ASSERT(0);
    }

    /* Initialize GATT Database */
    status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    if (WICED_BT_GATT_SUCCESS != status)
    {
        printf("GATT initialization failed because of error: %d \n", status);
        CY_ASSERT(0);
    }

    /* Start Undirected Bluetooth LE Advertisements on device startup.
     * The corresponding parameters are contained in 'app_bt_cfg.c' */
    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    if (WICED_BT_SUCCESS != result)
    {
        printf( "Advertisement cannot start because of error: %d \n",
                   result);
        CY_ASSERT(0);
    }

    /* Start tput timer */
    if (CY_RSLT_SUCCESS != cyhal_timer_start(&tput_timer_obj))
    {
        printf("Throughput timer start failed !");
        CY_ASSERT(0);
    }

}

/*
 Function name:
 tput_timer_callb

 Function Description:
 @brief  This callback function is invoked on timeout of 5 second timer.

 @param  void*: unused
 @param cyhal_timer_event_t: unused

 @return void
 */
void tput_timer_callb(void *callback_arg, cyhal_timer_event_t event)
{
    cy_rslt_t result;
    tput_fun = 1;

    result = cy_rtos_thread_set_notification(&tput_task_pointer);
    if (result != CY_RSLT_SUCCESS)
    {
        if (result == CY_RTOS_GENERAL_ERROR)
        {
            printf("CY_RTOS_GENERAL_ERROR thread notification failed !\n");
        }
        else if (result == CY_RTOS_BAD_PARAM)
        {
            printf("CY_RTOS_BAD_PARAM thread notification failed !\n");
        }
        else
        {
            printf("others thread notification failed !\n");
        }
    }
}

/*
 Function name:
 tput_task

 Function Description:
 @brief  This task calculates throughput every 5 seconds

 @param  cy_thread_arg_t: unused

 @return void
 */
void tput_task(cy_thread_arg_t arg){

    while(true){
        cy_rtos_thread_wait_notification(CY_RTOS_NEVER_TIMEOUT);
        /* Display GATT TX throughput result */
        if ((conn_state_info.conn_id) &&(app_throughput_measurement_notify_client_char_config[0]) && (gatt_notif_tx_bytes))
        {
            /*GATT Throughput=(number of bytes sent/received in 1 second*8 bits) bps*/
            gatt_notif_tx_bytes = (gatt_notif_tx_bytes * 8) / (5*1000);
            printf("GATT NOTIFICATION : Server Throughput (TX)= %lu kbps\n", gatt_notif_tx_bytes);
            /* Reset the GATT notification byte counter */
            gatt_notif_tx_bytes = 0;
        }
        /* Display GATT RX throughput result */
        if (conn_state_info.conn_id && gatt_write_rx_bytes)
        {
            /*GATT Throughput=(number of bytes sent/received in 1 second*8 bits ) bps*/
            gatt_write_rx_bytes = (gatt_write_rx_bytes * 8) / (5*1000);
            printf("GATT WRITE        : Server Throughput (RX)= %lu kbps\n", gatt_write_rx_bytes);
            /* Reset the GATT write byte counter */
            gatt_write_rx_bytes = 0;
        }
        cy_rtos_semaphore_set(&semaphore);
        tput_fun = 0;
    }
}

/*
 Function name:
 Notify_task

 Function Description:
 @brief  This task send notifications when the semaphore is set by sema_set_task

 @param  cy_thread_arg_t: unused

 @return void
 */
void notify_task(cy_thread_arg_t arg)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    while(true)
    {
        if(tput_fun == 1){
            cy_rtos_semaphore_get(&semaphore, CY_RTOS_NEVER_TIMEOUT);
        }
        if (app_throughput_measurement_notify_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION)
        {
            for(int i=0; i<PACKET_PER_EVENT; i++)
            {
                status = wiced_bt_gatt_server_send_notification(conn_state_info.conn_id,
                                                            HDLC_THROUGHPUT_MEASUREMENT_NOTIFY_VALUE,
                                                            NOTIFICATION_DATA_SIZE,
                                                            notification_data_seq, NULL);
                if(WICED_BT_GATT_SUCCESS == status)
                {
                    gatt_notif_tx_bytes += NOTIFICATION_DATA_SIZE;
                }
                 if(WICED_BT_GATT_CONGESTED == status)
                {
                    cy_rtos_semaphore_get(&congestion, CY_RTOS_NEVER_TIMEOUT);
                }
            }
            cy_rtos_delay_milliseconds(10);
        }
        else{
            cy_rtos_delay_milliseconds(100);
        }
    }
}

/**
 * Function Name:
 * app_bt_gatt_event_callback
 *
 * Function Description:
 * @brief  This Function handles the all the GATT events - GATT Event Handler
 *
 * @param event            Bluetooth LE GATT event type
 * @param p_event_data     Pointer to Bluetooth LE GATT event data
 *
 * @return wiced_bt_gatt_status_t  Bluetooth LE GATT status
 */
static wiced_bt_gatt_status_t app_bt_gatt_event_callback(wiced_bt_gatt_evt_t event,
                                                         wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    /* Call the appropriate callback function based on the GATT event type,
     * and pass the relevant event
     * parameters to the callback function */

    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = app_bt_connect_event_handler (&p_event_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        status = app_bt_server_event_handler (p_event_data);
        break;
        /* GATT buffer request, typically sized to max of bearer mtu - 1 */
    case GATT_GET_RESPONSE_BUFFER_EVT:
        p_event_data->buffer_request.buffer.p_app_rsp_buffer = app_bt_alloc_buffer(p_event_data->buffer_request.len_requested);
        p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_bt_free_buffer;
        status = WICED_BT_GATT_SUCCESS;
        break;
        /* GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
    case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
             pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free){
                pfn_free(p_event_data->buffer_xmitted.p_app_data);
            }
            status = WICED_BT_GATT_SUCCESS;
        }
        break;

    case GATT_CONGESTION_EVT:
        if(!p_event_data->congestion.congested)
        {
            cy_rtos_semaphore_set(&congestion);
        }
        status = WICED_BT_GATT_SUCCESS;
        break;

    default:

        status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return status;
}

/**
 * Function Name
 * app_bt_connect_event_handler
 *
 * Function Description
 * @brief   This callback function handles connection status changes.
 *
 * @param p_conn_status    Pointer to data that has connection details
 *
 * @return wiced_bt_gatt_status_t See possible status codes in wiced_bt_gatt_status_e
 * in wiced_bt_gatt.h
 */

static wiced_bt_gatt_status_t app_bt_connect_event_handler (wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_result_t result;

    if (NULL != p_conn_status)
    {
        if (p_conn_status->connected)
        {
            /* Device has connected */
            printf("Connected : BDA ");
            print_bd_address(p_conn_status->bd_addr);
            printf("Packets %d\n",PACKET_PER_EVENT);
            /* Store the connection ID and peer BD Address */
            conn_state_info.conn_id = p_conn_status->conn_id;

            printf("Connection ID:  %d\n",conn_state_info.conn_id);
            memcpy(conn_state_info.remote_addr, p_conn_status->bd_addr, BD_ADDR_LEN);

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_OFF_CONN_ON;
            /* Save BT connection ID in application data structure */
            conn_state_info.conn_id = p_conn_status->conn_id;
            /* Save BT peer ADDRESS in application data structure */
            memcpy(conn_state_info.remote_addr, p_conn_status->bd_addr, BD_ADDR_LEN);
            wiced_bt_ble_phy_preferences_t phy_preferences;

            phy_preferences.rx_phys = BTM_BLE_PREFER_2M_PHY;
            phy_preferences.tx_phys = BTM_BLE_PREFER_2M_PHY;
            memcpy(phy_preferences.remote_bd_addr, conn_state_info.remote_addr, BD_ADDR_LEN);

            result = wiced_bt_ble_set_phy(&phy_preferences);

            if (result != WICED_BT_SUCCESS)
            {
                printf("Failed to send request to switch PHY %d\n",result);
            }

            if (CY_RSLT_SUCCESS != cyhal_timer_start(&tput_timer_obj))
            {
                printf("Throughput timer start failed !");
                CY_ASSERT(0);
            }
        }
        else
        {
            /* Device has disconnected */
            printf("Disconnected : BDA ");
            print_bd_address(p_conn_status->bd_addr);
            printf("Connection ID '%d', Reason '%s'\n",p_conn_status->conn_id,
                                    get_bt_gatt_disconn_reason_name(p_conn_status->reason));

            /* Reset the connection information */
            memset(&conn_state_info, 0u, sizeof(conn_state_info));

            if (CY_RSLT_SUCCESS != cyhal_timer_stop(&tput_timer_obj))
            {
                printf("Throughput timer stop failed !");
                CY_ASSERT(0);
            }

            /* Restart the advertisements */
            result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
            if (WICED_BT_SUCCESS != result)
            {
                printf( "Advertisement cannot start because of error: %d \n", result);
                CY_ASSERT(0);
            }

            /* Update the adv/conn state */
            app_bt_adv_conn_state = APP_BT_ADV_ON_CONN_OFF;
        }

        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

/**
 * Function Name:
 * app_bt_server_event_handler
 *
 * Function Description:
 * @brief  The callback function is invoked when GATT_ATTRIBUTE_REQUEST_EVT occurs
 *         in GATT Event handler function. GATT Server Event Callback function.
 *
 * @param p_data   Pointer to Bluetooth LE GATT request data
 *
 * @return wiced_bt_gatt_status_t  Bluetooth LE GATT status
 */
static wiced_bt_gatt_status_t app_bt_server_event_handler (wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_attribute_request_t   *p_att_req = &p_data->attribute_request;

    switch (p_att_req->opcode)
    {
    /* Attribute read notification (attribute value internally read from GATT databatterye) */
    case GATT_REQ_READ:
    case GATT_REQ_READ_BLOB:
        status = app_bt_gatt_req_read_handler(p_att_req->conn_id,
                                            p_att_req->opcode,
                                            &p_att_req->data.read_req,
                                            p_att_req->len_requested);
        break;

    case GATT_REQ_READ_BY_TYPE:
        status = app_bt_gatt_req_read_by_type_handler(p_att_req->conn_id,
                                                  p_att_req->opcode,
                                                  &p_att_req->data.read_by_type,
                                                  p_att_req->len_requested);
        break;

    case GATT_REQ_WRITE:
    case GATT_CMD_WRITE:
    case GATT_CMD_SIGNED_WRITE:
        status = app_bt_write_handler(p_data);
        if ((p_att_req->opcode == GATT_REQ_WRITE) && (status == WICED_BT_GATT_SUCCESS))
        {
            wiced_bt_gatt_write_req_t *p_write_request = &p_att_req->data.write_req;
            wiced_bt_gatt_server_send_write_rsp(p_att_req->conn_id, p_att_req->opcode,
                                                p_write_request->handle);
        }
        break;

    case GATT_REQ_MTU:
    	printf("\rClient MTU: %d\n", p_att_req->data.remote_mtu);
        /* Application calls wiced_bt_gatt_server_send_mtu_rsp() with the desired mtu */
        status = wiced_bt_gatt_server_send_mtu_rsp(p_att_req->conn_id,
                                                   p_att_req->data.remote_mtu,
                                                   wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
        break;

    case GATT_HANDLE_VALUE_CONF:
        status = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_HANDLE_VALUE_NOTIF:
        status = WICED_BT_GATT_SUCCESS;
        break;

    default:
        status = WICED_BT_GATT_SUCCESS;
        break;
    }
    return status;
}

/**
 * Function Name:
 * app_bt_write_handler
 *
 * Function Description:
 * @brief   The function is invoked when GATTS_REQ_TYPE_WRITE is received from the
 *          client device and is invoked GATT Server Event Callback function. This
 *          handles "Write Requests" received from Client device.
 *
 * @param   p_write_req   Pointer to Bluetooth LE GATT write request
 *
 * @return  wiced_bt_gatt_status_t  Bluetooth LE GATT status
 */
static wiced_bt_gatt_status_t app_bt_write_handler(wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_write_req_t *p_write_req = &p_data->attribute_request.data.write_req;

    CY_ASSERT(( NULL != p_data ) && (NULL != p_write_req));

    return app_bt_set_value(p_write_req->handle,
                                    p_write_req->p_val,
                                    p_write_req->val_len);

}

/**
 * Function Name:
 * app_bt_set_value
 *
 * Function Description:
 * @brief  The function is invoked by app_bt_write_handler to set a value
 *         to GATT DB.
 *
 * @param attr_handle  GATT attribute handle
 * @param p_val        Pointer to Bluetooth LE GATT write request value
 * @param len          length of GATT write request
 *
 * @return wiced_bt_gatt_status_t  Bluetooth LE GATT status
 */
static wiced_bt_gatt_status_t app_bt_set_value(uint16_t attr_handle, uint8_t *p_val,
                                               uint16_t len)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        /* Check for a matching handle entry */
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in the external lookup table */
            if (app_gatt_db_ext_attr_tbl[i].max_len >= len)
            {
                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memset(app_gatt_db_ext_attr_tbl[i].p_data, 0x00, app_gatt_db_ext_attr_tbl[i].max_len);
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, app_gatt_db_ext_attr_tbl[i].cur_len);

                if (memcmp(app_gatt_db_ext_attr_tbl[i].p_data, p_val, app_gatt_db_ext_attr_tbl[i].cur_len) == 0)
                {
                    status = WICED_BT_GATT_SUCCESS;
                }

                if(app_gatt_db_ext_attr_tbl[i].handle == HDLD_THROUGHPUT_MEASUREMENT_NOTIFY_CLIENT_CHAR_CONFIG)
                {
                    if (GATT_CLIENT_CONFIG_NOTIFICATION == app_throughput_measurement_notify_client_char_config[0])
                    {
                        printf("Notifications Enabled\n");
                        gatt_write_rx_bytes = 0;
                    }
                    else
                    {
                        printf("Notifications Disabled\n");
                        gatt_notif_tx_bytes = 0;
                    }
                }
                if(app_gatt_db_ext_attr_tbl[i].handle == HDLC_THROUGHPUT_MEASUREMENT_WRITEME_VALUE)
                {
                    /* Receive GATT write commands from client
                        * and update the counter with number of
                        * bytes received.
                        */
                    gatt_write_rx_bytes += len;
                    status = WICED_BT_GATT_SUCCESS;
                }
            }
            else
            {
                /* Value to write will not fit within the table */
                status = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }
    if (WICED_BT_GATT_SUCCESS != status)
    {
        printf( "%s() FAILED %d \n", __func__, status);
    }
    return status;
}

/**
 * Function Name:
 * app_bt_find_by_handle
 *
 * Function Description:
 * @brief  Find attribute description by handle
 *
 * @param handle    handle to look up
 *
 * @return gatt_db_lookup_table_t   pointer containing handle data
 */
static gatt_db_lookup_table_t *app_bt_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}

/**
 * Function Name:
 * app_bt_gatt_req_read_handler
 *
 * Function Description:
 * @brief  This Function Process read request from peer device
 *
 * @param conn_id       Connection ID
 * @param opcode        Bluetooth LE GATT request type opcode
 * @param p_read_req    Pointer to read request containing the handle to read
 * @param len_requested length of data requested
 *
 * @return wiced_bt_gatt_status_t  Bluetooth LE GATT status
 */
static wiced_bt_gatt_status_t app_bt_gatt_req_read_handler(uint16_t conn_id,
                                                           wiced_bt_gatt_opcode_t opcode,
                                                           wiced_bt_gatt_read_t *p_read_req,
                                                           uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t attr_len_to_copy, to_send;
    uint8_t *from;

    if ((puAttribute = app_bt_find_by_handle(p_read_req->handle)) == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->cur_len;
    printf("read_handler: conn_id:%d Handle:%x offset:%d len:%d\n",
                conn_id, p_read_req->handle, p_read_req->offset, attr_len_to_copy);

    if (p_read_req->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    to_send = MIN(len_requested, attr_len_to_copy - p_read_req->offset);
    from = puAttribute->p_data + p_read_req->offset;
    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL); /* No need for context, as buff not allocated */
}

/**
 * Function Name:
 * app_bt_gatt_req_read_by_type_handler
 *
 * Function Description:
 * @brief  Process read-by-type request from peer device
 *
 * @param conn_id       Connection ID
 * @param opcode        LE GATT request type opcode
 * @param p_read_req    Pointer to read request containing the handle to read
 * @param len_requested length of data requested
 *
 * @return wiced_bt_gatt_status_t  LE GATT status
 */
static wiced_bt_gatt_status_t app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
                                                                   wiced_bt_gatt_opcode_t opcode,
                                                                   wiced_bt_gatt_read_by_type_t *p_read_req,
                                                                   uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t last_handle = 0;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    uint8_t pair_len = 0;
    int used_len = 0;

    if (NULL == p_rsp)
    {
        printf("No memory, len_requested: %d!!\r\n",len_requested);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        last_handle = attr_handle;
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle,
                                                        &p_read_req->uuid);
        if (0 == attr_handle )
            break;

        if ( NULL == (puAttribute = app_bt_find_by_handle(attr_handle)))
        {
            printf("found type but no attribute for %d \r\n",last_handle);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len, len_requested - used_len, &pair_len,
                                                                attr_handle, puAttribute->cur_len, puAttribute->p_data);
            if (0 == filled)
            {
                break;
            }
            used_len += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (0 == used_len)
    {
       printf("attr not found  start_handle: 0x%04x  end_handle: 0x%04x  Type: 0x%04x\r\n",
               p_read_req->s_handle, p_read_req->e_handle, p_read_req->uuid.uu.uuid16);
        app_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */

    return wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len, used_len, p_rsp, (void *)app_bt_free_buffer);
}

/* [] END OF FILE */
