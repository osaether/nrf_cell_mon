#include "nrfx_saadc.h"
#include "thread_utils.h"

#define VOLT_LEN        8
#define SAADC_CHANNEL   0
#define MAX_MILLIVOLT   6000

static volatile bool m_sample_ready;
static volatile bool m_calibrate_done;
static nrf_saadc_value_t m_sample;
static nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SAADC_CH_PSELP_PSELP_VDDHDIV5);
static nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
static nrf_saadc_value_t m_buffer;

static void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
    switch (p_event->type)
    {
        case NRFX_SAADC_EVT_DONE:
            m_sample = p_event->data.done.p_buffer[0];
            m_sample_ready = true;
            break;

        case NRFX_SAADC_EVT_CALIBRATEDONE:
            m_calibrate_done = true;
            break;

        default:
            ASSERT(true);
            break;
    }
}

static void saadc_init(void)
{
    nrfx_err_t err_code;
 
    err_code = nrfx_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);
    channel_config.gain = SAADC_CH_CONFIG_GAIN_Gain1_2;
    channel_config.burst = NRF_SAADC_BURST_ENABLED;
    err_code = nrfx_saadc_channel_init(SAADC_CHANNEL, &channel_config);
    APP_ERROR_CHECK(err_code);

    m_calibrate_done = false;
    
    err_code = nrfx_saadc_calibrate_offset();
    APP_ERROR_CHECK(err_code);

    while(!m_calibrate_done)
    {
        thread_process();
    }
}

uint32_t get_voltage(void)
{
    nrfx_err_t err_code;
    saadc_init();

    m_sample_ready = false;

    err_code = nrfx_saadc_buffer_convert(&m_buffer, 1);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_saadc_sample();
    APP_ERROR_CHECK(err_code);

    while(!m_sample_ready)
    {
        thread_process();
    }

    nrfx_saadc_uninit();
    return m_sample*MAX_MILLIVOLT/4096;
}