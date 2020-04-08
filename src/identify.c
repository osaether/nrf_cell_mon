#include "app_timer.h"
#include "bsp_thread.h"
#include "thread_utils.h"
#include "led_utils.h"

APP_TIMER_DEF(m_identify_timer);

static volatile bool m_bdone;
static bool m_binitialized = false;

static void identify_timer_handler(void * p_context)
{
    static int cnt = 0;
    switch(cnt++)
    {
        case 0:
            bsp_board_led_on(0);
            break;
        case 1:
            bsp_board_led_off(0);
            bsp_board_led_on(1);
            break;
        case 2:
            bsp_board_led_off(1);
            bsp_board_led_on(2);
            break;
        case 3:
            bsp_board_led_off(2);
            bsp_board_led_on(3);
            break;
        case 4:
            bsp_board_led_off(3);
            m_bdone = true;
            cnt = 0;
            break;
    }
}

void identify(void)
{
    ret_code_t err_code;

    if(!m_binitialized)
    {
        err_code = app_timer_create(&m_identify_timer,
                                    APP_TIMER_MODE_REPEATED,
                                    identify_timer_handler);
        APP_ERROR_CHECK(err_code);
        m_binitialized = true;
    }
    uint32_t leds_state = led_is_on(BSP_BOARD_LED_MASK_0 | BSP_BOARD_LED_MASK_1 | BSP_BOARD_LED_MASK_2 | BSP_BOARD_LED_MASK_3);
    leds_off(leds_state);
    m_bdone = false;
    err_code = app_timer_start(m_identify_timer, APP_TIMER_TICKS(500), NULL);
    APP_ERROR_CHECK(err_code);
    while(!m_bdone)
    {
        thread_process();
    }
    app_timer_stop(m_identify_timer);
    leds_on(leds_state);
}