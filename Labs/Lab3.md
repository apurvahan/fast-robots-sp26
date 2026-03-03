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
* [Lab Tasks](#labtasks)
* [Gyroscope](#gyroscope)
* [Sample Data](#sampledata)
* [Stunt](#stunt)

---

#prelab
The VL53L1X has a default 8-bit I²C address of 0x52. However, the least significant bit is a read-write bit which isn't super significant for our purposes. The Artemis I²C is a 7-bit address so we left shift the address from 0x52 to 0x26 (01010010 to 0-0101001). So, the default address of the VL53L1X is 0x26 address and since both sensors are on the same I²C bus, they will both automatically try to communicate on that address which causes communication issues. There is no way to distinguish each sensor from the other in hardware because the pinout is VDC, VIN, GND, SDA, SCL, XSHUT, and GPIO1. Unlike the IMU which had an AD0/AD1 option, the only way to distinguish the sensors from each other is to change the address in the code once they are plugged into the Artemis. 

So, in order to use the two sensors, one of them has to be assigned a different I²C address. The address itself can be found by scanning the bus and I found that 0x2A was open so I used that as my second I²C address. Since I can't turn both sensors on immediately because of the address conflict, I have to start by only having one active on the bus which I did by soldering the XSHUT pin on one of the TOF sensors to pin 8 on the Artemis and then writing it low. This kept the other sensor active and I began that sensor and assigned that sensor to 0x2A. Then, I wrote pin 8 on the Artemis to high which made the TOF sensor active and I began the sensor. Since no address was assigned to the second TOF sensor it defaulted to 0x29 which is fine because I had already set the first sensor to a different address. 

###Sensor Placement
There should be sensors at the front and the sides to prevent any sort of collisions to the front half of the car. We don't need TOF sensors in the back because we know where the car is if we know the orientation and the distance of the front to some obstacle. Since there are 2 TOF sensors, the best approach would be to put each one on the front left and front right, respectively, angled at around 45 degrees. This allows sensor fusion for obstacle detection of objects immediately in front of the robot because both sensors can see it while also allowing the robot to see on both sides. 

###Wiring Diagram
![Wiring Diagram](../Images/lab3_pinout.png)


#labtasks
![Serial Monitor of Time of Fight Sensors](.../Images/serial_monitor_TOF.png)

This is my serial monitor after having 2 TOF sensors and 1 IMU plugged in. 

```bash
Both sensors online!
Sensor A renamed to 0x2A
Sensor B booted at 0x29
Both sensors online!
Found device at 0x15
Found device at 0x29
Found device at 0x69
```

Code for scanning the bus:
```C++
for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(address, HEX);
    }
  }
```
###Distance Modes
The IMUs have 3 distance modes: short, medium, and long. The long range mode allows a ranging distance of 4m but is more sensitive to ambient light and/or other light related error because of how much of the environment it is taking in. The short range mode has a smaller distance of 1.3m but is less light sensitive because it doesn't have to range in a large area. For this lab and also generally for the robot I think short range is enough because 1.3 meters is quite a lot relative to the size of the robot. 

![Wiring Diagram](../Images/distance_table_TOF.png)