#include <nrf_temp.h>
#include "temperature.h"

int32_t temp_internal_read(void)
{
    NRF_TEMP->TASKS_START = 1;
    /* Busy wait while temperature measurement is not finished. */
    while (NRF_TEMP->EVENTS_DATARDY == 0)
    {
        // Do nothing.
    }
    NRF_TEMP->EVENTS_DATARDY = 0;

    int32_t temp = nrf_temp_read() / 4;

    NRF_TEMP->TASKS_STOP = 1;

    return temp;
}

void temp_init(void)
{
    nrf_temp_init();
}