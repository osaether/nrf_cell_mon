/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <app_timer.h>
#include <app_scheduler.h>
#include <bsp_thread.h>
#include <nrf_log.h>
#include <nrf_log_ctrl.h>
#include <nrf_log_default_backends.h>
#include <nrfx_wdt.h>
#include <mqttsn_client.h>
#include <thread_utils.h>
#include <openthread/thread.h>

#include "temperature.h"
#include "tmp102.h"
#include "voltage.h"
#include "identify.h"
#include "vdd_out.h"


#define SCHED_QUEUE_SIZE        32                                        // Maximum number of events in the scheduler queue
#define SCHED_EVENT_DATA_SIZE   APP_TIMER_SCHED_EVENT_DATA_SIZE           // Maximum app_scheduler event size
#define DEFAULT_CHILD_TIMEOUT   40                                        // Thread child timeout [s]
#define DEFAULT_POLL_PERIOD     1000                                      // Thread Sleepy End Device polling period when MQTT-SN Asleep [ms]
#define SHORT_POLL_PERIOD       100                                       // Thread Sleepy End Device polling period when MQTT-SN Awake [ms]
#define SEARCH_GATEWAY_TIMEOUT  5                                         // MQTT-SN Gateway discovery procedure timeout [s]
#define PUBLISH_INTERVAL        3000
#define TOPIC_LEN               MQTTSN_CLIENT_ID_MAX_LENGTH+16

const char                      *m_pname = "nrfcellmon";

static mqttsn_topic_t m_pub_topics[] =
{
    {(uint8_t *)"mv", 0},
    {( uint8_t *)"ts", 0},
    {( uint8_t *)"tb", 0},
};

#define PUB_TOPIC_MV      0
#define PUB_TOPIC_TS      1
#define PUB_TOPIC_TB      2


static mqttsn_topic_t m_sub_topics[] =
{
    {(uint8_t *)"identify", 0},
    {(uint8_t *)"reset", 0}
};

#define SUB_TOPIC_IDENT   0
#define SUB_TOPIC_RESET   1

APP_TIMER_DEF(m_publish_timer);

static int8_t                 m_battery_temp;
static int32_t                m_internal_temp;
static uint32_t               m_mvmin = 1e4;
static uint32_t               m_mvmax = 0;
static uint32_t               m_mvoltage = 0;
static volatile bool          m_mqtt_registered;
static mqttsn_client_t        m_client;                                   // An MQTT-SN client instance
static mqttsn_remote_t        m_gateway_addr;                             // A gateway address
static uint8_t                m_gateway_id;                               // A gateway ID
static mqttsn_connect_opt_t   m_connect_opt;                              // Connect options for the MQTT-SN client
static uint16_t               m_msg_id           = 0;                     // Message ID thrown with MQTTSN_EVENT_TIMEOUT
static char                   m_client_id[MQTTSN_CLIENT_ID_MAX_LENGTH];   // The MQTT-SN Client's ID
static char                   m_topic_name[TOPIC_LEN];
static uint16_t               m_topic_id;
static nrfx_wdt_channel_id    m_channel_id;


static void mqttsn_init(void * p_event_data, uint16_t event_size);


static void sleep(void)
{
    otError error;

    error = otLinkSetPollPeriod(thread_ot_instance_get(), DEFAULT_POLL_PERIOD);
    ASSERT(error == OT_ERROR_NONE);
}

static void wake_up(void)
{
    otError error;

    error = otLinkSetPollPeriod(thread_ot_instance_get(), SHORT_POLL_PERIOD);
    ASSERT(error == OT_ERROR_NONE);
}


static void mqttsn_connect(void * p_event_data, uint16_t event_size)
{
    uint32_t err_code = mqttsn_client_connect(&m_client, &m_gateway_addr, m_gateway_id, &m_connect_opt);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("CONNECT message could not be sent. Error: 0x%x\r\n", err_code);
        APP_ERROR_CHECK(err_code);
    }
}


static void gateway_info_callback(mqttsn_event_t * p_event)
{        
    m_gateway_addr = *(p_event->event_data.connected.p_gateway_addr);
    m_gateway_id = p_event->event_data.connected.gateway_id;    
    app_sched_event_put(NULL, 0, mqttsn_connect);
}


static void disconnected_callback(void)
{
     bsp_board_led_off(2);
    app_timer_stop(m_publish_timer);
    app_sched_event_put(NULL, 0, mqttsn_connect);
}


static void regack_callback(mqttsn_event_t * p_event)
{
    if (p_event->event_id != MQTTSN_EVENT_REGISTERED)
        return;
    if (p_event->event_data.registered.packet.id == m_msg_id)
    {
        m_topic_id = p_event->event_data.registered.packet.topic.topic_id;
        m_mqtt_registered = true;
    }
}


static void puback_callback(void)
{
    nrfx_wdt_channel_feed(m_channel_id);
    sleep();
}


static void sleep_callback(void)
{
    sleep();
}


static void wakeup_callback(void)
{
    NRF_LOG_INFO("wake_up()\r\n");
    wake_up();
}


static void timeout_callback(mqttsn_event_t * p_event)
{
    NRF_LOG_INFO("MQTT-SN event: Timed-out message: %d. Message ID: %d.\r\n",
                  p_event->event_data.error.msg_type,
                  p_event->event_data.error.msg_id);
    bsp_board_led_off(2);
    app_timer_stop(m_publish_timer);
    mqttsn_client_uninit(&m_client);
    app_sched_event_put(NULL, 0, mqttsn_init);
}


static void searchgw_timeout_callback(mqttsn_event_t * p_event)
{
    NRF_LOG_INFO("MQTT-SN event: Gateway discovery result: 0x%x.\r\n", p_event->event_data.discovery);
    sleep();
}


static void received_callback(mqttsn_event_t * p_event)
{
    if (p_event->event_data.published.packet.topic.topic_id == m_sub_topics[SUB_TOPIC_IDENT].topic_id)
    {
        identify();
    }
    else if (p_event->event_data.published.packet.topic.topic_id == m_sub_topics[SUB_TOPIC_RESET].topic_id)
    {
        NVIC_SystemReset();
    }
}

static void mqtt_register(void * p_event_data, uint16_t event_size)
{
    uint32_t err_code;
    int i;
    
    for(i=0;i<sizeof(m_pub_topics)/sizeof(mqttsn_topic_t);i++)
    {
        snprintf(m_topic_name, TOPIC_LEN, "%s/%s/%s", m_pname, m_client_id, m_pub_topics[i].p_topic_name);
        m_mqtt_registered = false;
        err_code = mqttsn_client_topic_register(&m_client, (uint8_t *)m_topic_name, strlen(m_topic_name), &m_msg_id);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("REGISTER message could not be sent. Error code: 0x%x\r\n", err_code);
        }
        while (!m_mqtt_registered)
        {
            thread_process();
        }
        m_pub_topics[i].topic_id = m_topic_id;
    }
    
    for(i=0;i<sizeof(m_sub_topics)/sizeof(mqttsn_topic_t);i++)
    {
        snprintf(m_topic_name, TOPIC_LEN, "%s/%s/%s", m_pname, m_client_id, m_sub_topics[i].p_topic_name);
        m_mqtt_registered = false;
        err_code = mqttsn_client_topic_register(&m_client, (uint8_t *)m_topic_name, strlen(m_topic_name), &m_msg_id);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("REGISTER message could not be sent. Error code: 0x%x\r\n", err_code);
        }
        while (!m_mqtt_registered)
        {
            thread_process();
        }
        m_sub_topics[i].topic_id = m_topic_id;
        err_code = mqttsn_client_subscribe(&m_client, (uint8_t *)m_topic_name, strlen(m_topic_name), &m_msg_id);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("SUBSCRIBE message could not be sent.\r\n");
        }
    }
    err_code = app_timer_start(m_publish_timer, APP_TIMER_TICKS(PUBLISH_INTERVAL), NULL);
    APP_ERROR_CHECK(err_code);
}

static void connected_callback(void)
{
    bsp_board_led_on(2);
    app_sched_event_put(NULL, 0, mqtt_register);
}


void mqttsn_evt_handler(mqttsn_client_t * p_client, mqttsn_event_t * p_event)
{
    switch (p_event->event_id)
    {
        case MQTTSN_EVENT_GATEWAY_FOUND:
            NRF_LOG_INFO("MQTT-SN event: Client has found an active gateway.\r\n");
            gateway_info_callback(p_event);
            break;

        case MQTTSN_EVENT_CONNECTED:
            NRF_LOG_INFO("MQTT-SN event: Client connected.\r\n");
            connected_callback();
            break;

        case MQTTSN_EVENT_DISCONNECTED:
            NRF_LOG_INFO("MQTT-SN event: Client disconnected.\r\n");
            disconnected_callback();
            break;

        case MQTTSN_EVENT_REGISTERED:
            NRF_LOG_INFO("MQTT-SN event: Client registered topic.\r\n");
            regack_callback(p_event);
            break;

        case MQTTSN_EVENT_PUBLISHED:
            NRF_LOG_INFO("MQTT-SN event: Client has successfully published content.\r\n");
            puback_callback();
            break;

        case MQTTSN_EVENT_SLEEP_PERMIT:
            NRF_LOG_INFO("MQTT-SN event: Client permitted to sleep.\r\n");
            sleep_callback();
            break;

       case MQTTSN_EVENT_RECEIVED:
            NRF_LOG_INFO("MQTT-SN event: Client received content.\r\n");
            received_callback(p_event);
            break;

        case MQTTSN_EVENT_SLEEP_STOP:
            NRF_LOG_INFO("MQTT-SN event: Client wakes up.\r\n");
            wakeup_callback();
            break;

        case MQTTSN_EVENT_TIMEOUT:
            NRF_LOG_INFO("MQTT-SN event: Retransmission retries limit has been reached.\r\n");
            timeout_callback(p_event);
            break;

        case MQTTSN_EVENT_SEARCHGW_TIMEOUT:
            NRF_LOG_INFO("MQTT-SN event: Gateway discovery procedure has finished.\r\n");
            searchgw_timeout_callback(p_event);
            break;

        default:
            break;
    }
}

static void mqttsn_init(void * p_event_data, uint16_t event_size)
{
    uint32_t err_code = mqttsn_client_init(&m_client,
                                           MQTTSN_DEFAULT_CLIENT_PORT,
                                           mqttsn_evt_handler,
                                           thread_ot_instance_get());
    APP_ERROR_CHECK(err_code);
    m_connect_opt.alive_duration = MQTTSN_DEFAULT_ALIVE_DURATION;
    m_connect_opt.clean_session  = MQTTSN_DEFAULT_CLEAN_SESSION_FLAG;
    m_connect_opt.will_flag      = MQTTSN_DEFAULT_WILL_FLAG;
    
    snprintf(m_client_id, MQTTSN_CLIENT_ID_MAX_LENGTH, "%08lX%08lX", NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0]);
    m_connect_opt.client_id_len  = strlen(m_client_id);

    memcpy(m_connect_opt.p_client_id, (unsigned char *)m_client_id, m_connect_opt.client_id_len);
    printf("Client-ID:%s\r\n", m_client_id);
}

static void state_changed_callback(uint32_t flags, void * p_context)
{
    uint32_t err_code;
    if ((flags & OT_CHANGED_THREAD_ROLE) != 0)
    {
        otDeviceRole role = otThreadGetDeviceRole(thread_ot_instance_get());
        switch(role)
        {
            case OT_DEVICE_ROLE_CHILD:
            case OT_DEVICE_ROLE_ROUTER:
            case OT_DEVICE_ROLE_LEADER:   
                err_code = mqttsn_client_search_gateway(&m_client, SEARCH_GATEWAY_TIMEOUT);
                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_ERROR("SEARCH GATEWAY message could not be sent. Error: 0x%x\r\n", err_code);
                    APP_ERROR_CHECK(err_code);
                }
             default:
                break;
        }
    }
    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}

static void publish(void * p_event_data, uint16_t event_size)
{
    uint32_t err_code;

    err_code = mqttsn_client_publish(&m_client, m_pub_topics[PUB_TOPIC_TB].topic_id, (const uint8_t *)&m_battery_temp, 1, &m_msg_id);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("PUBLISH message could not be sent. Error code: 0x%x\r\n", err_code)
    }
    err_code = mqttsn_client_publish(&m_client, m_pub_topics[PUB_TOPIC_TS].topic_id, (const uint8_t *)&m_internal_temp, 1, &m_msg_id);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("PUBLISH message could not be sent. Error code: 0x%x\r\n", err_code)
    }
    err_code = mqttsn_client_publish(&m_client, m_pub_topics[PUB_TOPIC_MV].topic_id, (const uint8_t *)&m_mvoltage, 4, &m_msg_id);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("PUBLISH message could not be sent. Error code: 0x%x\r\n", err_code)
    }
    uint32_t mvround = (m_mvoltage+25)/50;
    mvround = mvround * 50;
    NRF_LOG_INFO("m_mvoltage=%d, mvround=%d, m_mvmin=%d, m_mvmax=%d", m_mvoltage, mvround, m_mvmin, m_mvmax);
}

static void sample(void * p_event_data, uint16_t event_size)
{
    m_mvoltage = get_voltage();
    if (m_mvoltage < m_mvmin)
        m_mvmin = m_mvoltage;
    if (m_mvoltage > m_mvmax)
        m_mvmax = m_mvoltage;
    m_internal_temp = temp_internal_read();
    m_battery_temp = tmp102_read();
    app_sched_event_put(NULL, 0, publish);
}

static void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            NVIC_SystemReset();
            break;

        default:
            break;
   }
}


static void publish_timer_handler(void * p_context)
{
    wake_up();
    app_sched_event_put(NULL, 0, sample);
}


static void timer_init(void)
{
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_publish_timer,
                                  APP_TIMER_MODE_REPEATED,
                                  publish_timer_handler);
    APP_ERROR_CHECK(err_code);
}


static void leds_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


static void thread_bsp_init(void)
{
    uint32_t err_code;
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_thread_init(thread_ot_instance_get());
    APP_ERROR_CHECK(err_code);
}


static void thread_instance_init(void)
{
    thread_configuration_t thread_configuration =
    {
        .radio_mode            = THREAD_RADIO_MODE_RX_OFF_WHEN_IDLE,
        .autocommissioning     = true,
        .poll_period           = DEFAULT_POLL_PERIOD,
        .default_child_timeout = DEFAULT_CHILD_TIMEOUT,
    };

    thread_init(&thread_configuration);
    thread_state_changed_callback_set(state_changed_callback);
}

void wdt_event_handler(void)
{
}


static void wdt_init(void)
{
    nrfx_wdt_config_t config = NRFX_WDT_DEAFULT_CONFIG;
    uint32_t err_code = nrfx_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrfx_wdt_enable();
}

#if defined (BOARD_PCA10059)
static void gpio_output_voltage_setup(void)
{
    if (NRF_POWER->MAINREGSTATUS &
       (POWER_MAINREGSTATUS_MAINREGSTATUS_High << POWER_MAINREGSTATUS_MAINREGSTATUS_Pos))
    {
        if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) != (UICR_REGOUT0_VDD_OUT << UICR_REGOUT0_VOUT_Pos))
        {
            uint32_t nrffw[2];

            // Save bootloader address:
            nrffw[0] = NRF_UICR->NRFFW[0];
            nrffw[1] = NRF_UICR->NRFFW[1];

            NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

            NRF_NVMC->ERASEUICR = NVMC_ERASEUICR_ERASEUICR_Erase << NVMC_ERASEUICR_ERASEUICR_Pos;   //erase UICR
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

            NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

            NRF_UICR->NRFFW[0] = nrffw[0];
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            NRF_UICR->NRFFW[1] = nrffw[1];
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

            NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                                 (UICR_REGOUT0_VDD_OUT << UICR_REGOUT0_VOUT_Pos);

            NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

            // System reset is needed to update UICR registers.
            NVIC_SystemReset();
        }
    }
}
#endif


static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


int main(int argc, char *argv[])
{
    // Set VVD OUT to the voltage defined in vdd_out.h.
    // For nRF52833 and nRF52840 the voltage is set in
    // uicr.c
#if defined (BOARD_PCA10059)
    gpio_output_voltage_setup();
#endif
    log_init();
    scheduler_init();
    timer_init();
    temp_init();
    tmp102_init();
    leds_init();
    wdt_init();
    thread_instance_init();
    thread_bsp_init();
    mqttsn_init(NULL, 0);

    while(true)
    {
        thread_process();
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
            thread_sleep();
        }
    }
}

