#  Triple-axis Accelerometer

Author: Briana Zhao

Date: 2021-03-20
-----

## Summary

I connected the ADXL343 accelerometer to my ESP32 on the breadboard and used it to read acceleration in the x, y, and z axes. To do this, I used the provided base code for I2C and added the necessary modules to be able to access the necessary values. Then I wrote a module to read acceleration in the 3 axes, as well as a module to convert acceleration values into tilt data. The completed code reads values from the accelerometer and prints readings and tilt angles to the console.

## Sketches and Photos

Here is an image of the accelerometer hooked up to the ESP32:

<img src = "/skills/cluster-3/23/images/accelerometer.jpg">

Here is an image of the acceleration values and tilt data being read and printed to the console:

<img src = "/skills/cluster-3/23/images/console.png">


## Modules, Tools, Source Used Including Attribution

[I2C Base Code](https://github.com/BU-EC444/code-examples/tree/master/i2c-accel)
[Source for tilt data](https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing)
[Espressif I2C Info](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html)


## Supporting Artifacts

[Link to code](https://github.com/BU-EC444/Zhao-Briana/blob/master/skills/cluster-3/23/code/i2c_accel.c)


-----
