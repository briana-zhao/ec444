#  Building a Stopwatch

Author: Briana Zhao

Date: 2021-02-18
-----

## Summary

For this skill, I used an ESP32 and its built-in timer functionality to implement a stopwatch that counts up in seconds. The stopwatch starts counting after the button is pressed. The current count is displayed on the alphanumeric display (modulo 100), and the count resets at the press of a button. To do this, I added code to the provided 'timer_group' example and reused my code from skill 8 (Alphanumeric Display). It uses interrupts, both for the button and for the timer.


## Sketches and Photos

Here is an image of the stopwatch:

<img src="/skills/cluster-1/11/images/stopwatch.jpg">

Here is a link to a video of the stopwatch working:
https://drive.google.com/file/d/1CgurG-L121dD5SdAmo6UahQA7SgWBbtG/view?usp=sharing


## Modules, Tools, Source Used Including Attribution

I referenced the provided 'timer_group', 'button-timer' and 'i2c_display' example.


## Supporting Artifacts

Link to code: [here](https://github.com/BU-EC444/Zhao-Briana/blob/master/skills/cluster-1/11/code/stopwatch.c)


-----
