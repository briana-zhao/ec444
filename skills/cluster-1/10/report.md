#  RTOS TASKs – Free RTOS

Author: Briana Zhao

Date: 2021-02-17
-----

## Summary

I designed three FreeRTOS tasks using the ESP32, alphanumeric display, LEDs, and a button. In one task, the LEDs are used to count up and down in binary from 0 to 15. In another task, the direction of the counting is reversed when the button is pushed. This task implemented the button using a hardware interrupt. In the last task, 'UP' or 'DOWN' is displayed on the alphanumeric display depending on the direction of the counting. To do this, I added to the code I already had for skill 7 (GPIO) and skill 8 (Alphanumeric Display).


## Sketches and Photos

Here is an image of the RTOS counting down:

<img src="/skills/cluster-1/10/images/rtos1.jpg" width="40%">

Here is an image of the RTOS counting up:

<img src="/skills/cluster-1/10/images/rtos2.jpg" width="40%">

Here is a link to a video:
https://drive.google.com/file/d/1Gc-cH0RKFeyEg3DBn_FghilL2LR9DLOo/view?usp=sharing


## Modules, Tools, Source Used Including Attribution

I used the class example 'button-timer' to help implement the button task.


## Supporting Artifacts

Link to code: [here](https://github.com/BU-EC444/Zhao-Briana/blob/master/skills/cluster-1/10/code/rtos.c)


-----
