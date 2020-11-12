# Monitor for Lithium battery cells

This battery cell monitor uses the Nordic Semiconductor [nRF52840](https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF52840)/[nRF52833](https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF52833) with MQTT-SN over OpenThread.
It is meant to be used for monitoring a Lithium battery cell connected between the VDDH pin and ground on the nRF chip.

Three values are sampled and published on MQTT:

1. The voltage on the VDDH pin of the nRF chip
2. The internal temperature of the nRF chip
3. An external (optional) [TMP102](https://www.sparkfun.com/products/13314) temperature sensor mounted on the battery cell

# MQTT-SN

To identify each battery cell, the device ID is included in the topic. For now the topic to subscribe is "nrfcellmon/{ID}/data". One topic is published containing the device ID ("id"). The battery/cell voltage in mV ("mv"), the battery/cell temperature in degrees Celsius ("tb") and the self/device temperature in degrees Celsius ("ts") in the JSON encoded payload. If the device ID is "E19928773B8398AF", the battery voltage 3779mV, the battery temperature 22&deg;C and the internal temperature 25&deg;C, the published topic looks like this:

nrfcellmon/E19928773B8398AF/data

and the JSON encoded data:
{
  "mv" : 3779,
  "tb" : 22,
  "ts" : 25
}

Commands over MQTT

Commands can be sent to each cell moditor using the topic "nrfcellmon/device ID/command"

1.  identify

    This is used for device identification. When received the four LEDs of the development board are blinked in order.

    example "nrfcellmon/E19928773B8398AF/identify"

2.  reset

    Resets the device by calling NVIC_SystemReset

    example "nrfcellmon/E19928773B8398AF/reset"

# Note

The project files in this repo use relative paths to the nRF5 SDK. It should build out of the box when cloned in the `examples/thread` folder of the nRF5 SDK for Thread and Zigbee.

nRF5 SDK for Thread and Zigbee v4.1 and lower does not support using VDDH as input to the SAADC and you need to comment out these lines in the function `nrfx_saadc_channel_init` in the file `modules/nrfx/drivers/src/nrfx_saadc.c` to avoid getting an assert:
```
NRFX_ASSERT((p_config->pin_p <= NRF_SAADC_INPUT_VDD) &&
                (p_config->pin_p > NRF_SAADC_INPUT_DISABLED));
NRFX_ASSERT(p_config->pin_n <= NRF_SAADC_INPUT_VDD);
```