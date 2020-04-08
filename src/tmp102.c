#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "tmp102.h"
#include "thread_utils.h"

#if defined(BOARD_PCA10056) || defined(BOARD_PCA10100)
#define TMP102_SCL_PIN            26
#define TMP102_SDA_PIN            27
#define TMP102_VCC_PIN            2
#elif defined(BOARD_PCA10059)
#define TMP102_SCL_PIN            13
#define TMP102_SDA_PIN            15
#define TMP102_VCC_PIN            17
#endif


#define TMP102_TEMP_READ_REG_ADR  0x00
#define TMP102_ADR                (0x9<<3)

#define SEND_STOP_CONDITON        0

#define TWI_INSTANCE_ID           0
static const                      nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static volatile bool              m_xfer_done;


void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

static void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = TMP102_SCL_PIN,
       .sda                = TMP102_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
}


int8_t tmp102_read(void)
{
    uint8_t temperature_read_buffer[2];
    int16_t temperature_result = 0;
    uint8_t tmp_read_reg_adr = TMP102_TEMP_READ_REG_ADR;

    uint32_t err_code;
    nrf_drv_twi_enable(&m_twi);
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, TMP102_ADR, &tmp_read_reg_adr, 1, SEND_STOP_CONDITON);
    if(err_code != NRF_SUCCESS) return 0;
    while (!m_xfer_done)
        ;
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, TMP102_ADR, temperature_read_buffer, 2);
    if(err_code != NRF_SUCCESS) return 0;
    while (!m_xfer_done)
        ;
    nrf_drv_twi_disable(&m_twi);

    temperature_result = (temperature_read_buffer[0] << 4) | (temperature_read_buffer[1] >> 4);

    //The tmp102 does twos compliment but has the negative bit in the wrong spot, so test for it and correct if needed
    if(temperature_result & (1<<12))
        temperature_result |= 0xF800; //Set bits 12 to 15 to 1s to get this reading into real twos compliment

    return (int8_t)((temperature_result*625+5000)/10000);
}


void tmp102_init(void)
{
    //nrf_gpio_pin_clear(TMP102_VCC_PIN);
    nrf_gpio_pin_set(TMP102_VCC_PIN);
    nrf_gpio_cfg_output(TMP102_VCC_PIN);
    twi_init();
}

void tmp102_enable(void)
{
    nrf_gpio_pin_set(TMP102_VCC_PIN);
}

void tmp102_disable(void)
{
    nrf_gpio_pin_clear(TMP102_VCC_PIN);
}