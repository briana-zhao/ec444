#  Use PWM to control power delivery to LEDs

Author: Briana Zhao

Date: 2021-03-20
-----

## Summary

For this skill I used PWM to control an LED connected to the ESP32. I built upon the provided ledc example and allowed the user to either select an intensity for the LED or cycle through the range of intensities. If the user enters an integer from 0-9, then the LED will be set to the associated intensity level. If the user enters 'Cycle', then the LED cycles up then down the levels of intensity, staying in each step for 250 ms.

## Sketches and Photos

Here is an image of the circuit:

<img src = "/skills/cluster-3/24/images/ledcircuit.jpg">

Here is an image of the console:

<img src = "/skills/cluster-3/24/images/console.png">

Here is a GIF of the LED intensity being set with integer values 0-9:

<img src = "/skills/cluster-3/24/images/levels.gif">

Here is a GIF of the LED intensity being cycled:

<img src = "/skills/cluster-3/24/images/cycle.gif">



## Modules, Tools, Source Used Including Attribution

[Used provided ledc example](https://github.com/espressif/esp-idf/blob/73db142403c6e5b763a0e1c07312200e9b622673/examples/peripherals/ledc/main/ledc_example_main.c)

## Supporting Artifacts

[Link to code](https://github.com/BU-EC444/Zhao-Briana/blob/master/skills/cluster-3/24/code/ledc_pwm.c)


-----
