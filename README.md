# Monitor for Lithium battery cells

This battery cell monitor uses the Nordic Semiconductor [nRF52840](https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF52840)/[nRF52833](https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF52833) with MQTT-SN over OpenThread.
It is meant to be used for monitoring a Lithium battery cell connected between the VDDH pin and ground on the nRF chip.

Three values are sampled and published on MQTT:

1. The voltage on the VDDH pin of the nRF chip
2. The internal temperature of the nRF chip
3. An external (optional) [TMP102](https://www.sparkfun.com/products/13314) temperature sensor mounted on the battery cell

# MQTT-SN

To identify the cell monitor the device ID is used as part of the MQTT topic. One JSON topic is published containing the device ID ("id"), the battery/cell voltage in mV ("mv"), the battery/cell temperature in degrees Celsius ("tb") and the self/device temperature in degrees Celsius ("ts"). If the device ID is
"E19928773B8398AF", the battery voltage 3779mV, the battery temperature 22&deg;C and the internal temperature 25&deg;C, the published topic looks like this:

{
  "id" : "E19928773B8398AF",
  "mv" : 3779,
  "tb" : 22,
  "ts" : 25
}

The cell monitor subscribes to the following topics

1.  identify  
    This is used for device identification. When received the four LEDs of the development board are blinked in order.
2.  reset  
    Resets the device by calling NVIC_SystemReset