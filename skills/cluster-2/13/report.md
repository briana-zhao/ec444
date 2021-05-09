#  The Thermistor

Author: Briana Zhao

Date: 2021-02-28
-----

## Summary

I created a voltage divider with a thermistor (encapsulated 'waterproof' one) and a 1 kOhm resistor. I used the ADC pin from the ESP32 to read the voltage and convert this reading into a temperature reading in Celsius. To do this conversion, I first found the resistance across the thermistor using the equation R_t = R_o((V_s/V_o) - 1. Then I used the equation 1/T = 1/TO + (1/β) ⋅ ln (R/RO) to find the temperature. I used a B value of 3435. The reading is taken every 2 seconds and is displayed to the console. 


## Sketches and Photos

Here is a photo of the circuit:
<img src = "/skills/cluster-2/13/images/thermistorCircuit.jpg" width = "45%">

Here is a photo of the console:
<img src = "/skills/cluster-2/13/images/thermistor.png">


## Modules, Tools, Source Used Including Attribution

I used the following resources to learn how to convert a voltage reading into a temperature value:

https://www.digikey.com/en/maker/projects/how-to-measure-temperature-with-an-ntc-thermistor/4a4b326095f144029df7f2eca589ca54

https://www.jameco.com/Jameco/workshop/TechTip/temperature-measurement-ntc-thermistors.html


## Supporting Artifacts

[Link to my code](https://github.com/BU-EC444/Zhao-Briana/blob/master/skills/cluster-2/13/code/thermistor.c)


-----
