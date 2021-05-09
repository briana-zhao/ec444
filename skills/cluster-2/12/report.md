#  Battery Voltage Monitor

Author: Briana Zhao

Date: 2021-02-28
-----

## Summary

I used the ADC pin on the ESP32 to read voltages from a circuit. I created a voltage divider out of a 1 kOhm and a 220 Ohm resistor, and I provided voltage to it from the 3V pin on the ESP32. The ADC pin reads the voltage 10 times per second then it finds the average reading and displays it to the alphanumeric display. I set the attenuation rate to level 3, which extends the range of measurement to about 2600 mV.

## Sketches and Photos

Here is a photo of the circuit:

<img src = "/skills/cluster-2/12/images/batteryVoltage.jpg" width = "45%">

Here are also some readings from the console:

<img src = "/skills/cluster-2/12/images/batteryVoltageConsole.png">


## Modules, Tools, Source Used Including Attribution

[I referenced the provided 'adc' example.](https://github.com/espressif/esp-idf/tree/39f090a4f1dee4e325f8109d880bf3627034d839/examples/peripherals/adc)

## Supporting Artifacts

[Link to my code](https://github.com/BU-EC444/Zhao-Briana/blob/master/skills/cluster-2/12/code/batteryvoltage.c)


-----
