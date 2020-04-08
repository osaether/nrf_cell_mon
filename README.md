# nrf_cell_mon

Battery cell monitor for the Nordic Semiconductor [nRF52840](https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF52840)/[nRF52833](https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF52833) with MQTT-SN over OpenThread.
Three values are sampled and publised on MQTT:

1. The voltage on the VDDH pin of the nRF chip
2. The internal temperature of the nRF chip
3. An external [TMP102](https://www.sparkfun.com/products/13314) temperature sensor mounted on the battery cell

# MQTT-SN

To identify the cell monitor the device ID is used as part of the MQTT topic. Three topics are publised; the battery/cell voltage in mV ("mv"),
the battery/cell temperature in degrees Celsius ("tb") and the self/device temperature in degrees Celsius ("ts"). If the device ID is
"E19928773B8398AF", the three topics looks like this:

    nrfcellmon/E19928773B8398AF/mv
    nrfcellmon/E19928773B8398AF/tb
    nrfcellmon/E19928773B8398AF/ts

The cell monitor subscribes to the following topics

1.  identify  
    This is used for device identification. When received the four LEDs of the development board are blinked in order.
2.  reset  
    Resets the device by calling NVIC_SystemReset