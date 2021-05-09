#  Ultrasonic Range Sensor

Author: Briana Zhao

Date: 2021-02-28
-----

## Summary

I used an Ultrasonic Range Sensor with an ESP32 to measure distance and print it to the console in meters. To do this, I used the ADC pin on the ESP32 to read from the analog output of the Ultrasonic Range Sensor. To convert the reading into a distance in meters, I first found the Volts/Inch by taking the source voltage (3.3V) and dividing by 512. I then convert this to mVolts/Inch by multiplying by 1000. Then, I divide the reading from the Ultrasonic Range Sensor by this value to obtain a distance in inches. Lastly, this is converted to meters. A distance is determined every 2 seconds and is printed to the console. 


## Sketches and Photos
Here is a photo of the circuit:
<img src = "/skills/cluster-2/14/images/ultrasonic.jpg" width = "45%">

Here is a photo of the console:
<img src = "/skills/cluster-2/14/images/ultrasonicConsole.png">


## Modules, Tools, Source Used Including Attribution
These are the resources I used to learn how to convert the analog reading into distance in meters:

https://www.instructables.com/Max-Sonar-EZ0/

https://www.maxbotix.com/tutorials6/032-using-analog-voltage-pin-3.htm

https://www.maxbotix.com/documents/HRLV-MaxSonar-EZ_Datasheet.pdf


## Supporting Artifacts

[Link to my code](https://github.com/BU-EC444/Zhao-Briana/blob/master/skills/cluster-2/14/code/ultrasonic.c)


-----
