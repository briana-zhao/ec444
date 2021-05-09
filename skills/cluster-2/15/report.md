#  Long Range IR Range Sensor

Author: Briana Zhao

Date: 2021-02-28
-----

## Summary

I used a Long Range IR Range Sensor to measure distance in meters and display it to the console. This range sensor measure distances from 20 to 150 cm. I used the USB power pin to power the range sensor, and I used the ADC pin from the ESP32 to take its readings. To convert this reading into meters, I generated an equation using Excel from the values provided in the datasheet. This equation is y = 60.367x^-1.173, where x is the voltage reading and y is the distance in centimeters. The distance is then converted from centimeters to meters before being displayed on the console. A reading is taken every 2 seconds.


## Sketches and Photos
Here is a photo of the circuit:
<img src = "/skills/cluster-2/15/images/ir_rangefinder.jpg" width = "45%">

Here is a photo of the console:
<img src = "/skills/cluster-2/15/images/ir_rangefinder_console.png">

Here is a photo of the Excel spreadsheet used to generate the equation:
<img src = "/skills/cluster-2/15/images/spreadsheet.png" width = "75%">



## Modules, Tools, Source Used Including Attribution
I used this datasheet to acquire the specifics for the range sensor:

https://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf


## Supporting Artifacts

[Link to my code](https://github.com/BU-EC444/Zhao-Briana/blob/master/skills/cluster-2/15/code/rangefinder.c)

-----
