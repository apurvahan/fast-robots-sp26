---
layout: default
title: "Lab 2"
permalink: /LAB-2/
description: "Writeup for Lab 2."
---

<script>
window.MathJax = {
  tex: {
    inlineMath: [['$', '$'], ['\\(', '\\)']],
    displayMath: [['$$','$$'], ['\\[','\\]']]
  }
};
</script>
<script src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>

[← Back to Home]({{ '/' | relative_url }})

## Contents
* [Prelab Tasks](#prelab)
* [Accelerometer](#accelerometer)
* [Gyroscope](#gyroscope)
* [Sample Data](#sampledata)
* [Stunt](#stunt)

---

#prelab
The VL53L1X has a default 7-bit I²C address of 0x29. So, if both sensors are connected to the same I²C bus, they will attempt to respond at the same time which causes communication issues. There is no way to distinguish each sensor from the other in hardware because the pinout is VDC, VIN, GND, SDA, SCL, XSHUT, and GPIO1. Unlike the IMU which had an AD0/AD1 option, the only way to distinguish the sensors from each other is to change the address in the code once they are plugged into the Artemis. 

So, in order to use the two sensors, one of them has to be assigned a different I²C address. The address itself can be found by scanning the bus and I found that 0x2A was open so I used that as my second I²C address. Since I can't turn both sensors on immediately because of the address conflict, I have to start by only having one active on the bus which I did by soldering the XSHUT pin on one of the TOF sensors to pin 8 on the Artemis and then writing it low. This kept the other sensor active and I began that sensor and assigned that sensor to 0x2A. Then, I wrote pin 8 on the Artemis to high which made the TOF sensor active and I began the sensor. Since no address was assigned to the second TOF sensor it defaulted to 0x29 which is fine because I had already set the first sensor to a different address. 

###Sensor Placement
There should be sensors at the front and the sides to prevent any sort of collisions to the front half of the car. We don't need TOF sensors in the back because we know where the car is if we know the orientation and the distance of the front to some obstacle. Since there are 2 TOF sensors, the best approach would be to put each one on the front left and front right, respectively, angled at around 45 degrees. This allows sensor fusion for obstacle detection of objects immediately in front of the robot because both sensors can see it while also allowing the robot to see on both sides. 

###Wiring Diagram
