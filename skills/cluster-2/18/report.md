#  Raspberry Pi

Author: Briana Zhao

Date: 2021-03-04
-----

## Summary

For this skill I used a Raspberry Pi Zero W and connected it to a wireless network and hosted a Node.js web server. I first flashed the Raspberry Pi OS onto a  microSD card. Then I modified the files on the microSD card to enable ssh and allow the Raspberry Pi Zero W to connect to the WiFi. This step was challenging for me because I had to input the correct information for my router and determine the correct IP address for the Pi Zero. This took quite a bit of trial and error. Once I got the Pi Zero connected, I was able to ssh into it and install Node.js. After this I was able to open a web server with 'Hello, world!'.


## Sketches and Photos

Here is a photo of Node.js installed on the Pi Zero and hello.js being run:

<img src = "/skills/cluster-2/18/images/raspberrypi.png">

Here is a photo of the web server through the Pi Zero:

<img src = "/skills/cluster-2/18/images/webserver.png">


## Modules, Tools, Source Used Including Attribution

Useful source for downloading Node.js onto the Pi Zero: https://www.digitalocean.com/community/tutorials/how-to-install-node-js-on-ubuntu-20-04


## Supporting Artifacts

[node.js code used](https://github.com/BU-EC444/Zhao-Briana/blob/master/skills/cluster-2/18/code/hello.js)


-----
