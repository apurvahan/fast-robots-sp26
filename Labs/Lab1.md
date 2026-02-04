---
layout: default
title: "Lab 1"
permalink: /LAB-1/
description: "Writeup for Lab 1."
---

[← Back to Home]({{ '/' | relative_url }})

### Contents
* [Prelab](#prelab)
* [Lab Tasks](#labtasks)
* [Discussion](#discussion)

---

# prelab
For the most part, I followed the instructions in Lab 1 exactly. In order to activate my virtual environment for this class however, I found that I had to use
```bash
    Set-ExecutionPolicy Unrestricted -Scope Process
```
in order to give it the permissions to run.

I already had both python and pip installed and added to pip so I just updated those and then installed the packages and Arduino libraries.  

Blink
![blink GIF](Images/blink.gif)

<img src="Images/blink.gif" alt="Blinking LED" width="480" />


Arduino:
Running the Arduino scripts involved:
1. Opening up the example code required (ex Blink)
2. Connecting my board to it by choosing the right COM port (in my case COM6)
3. Opening the serial monitor in Arduino
4. Changing the baud rate in code to match up with my COM ports. Mine are on 9600 baud so I changed the example code to match that. Later on in the lab I changed the baud rate of my port instead.

MAC Address:
```bash
    Advertising BLE with MAC: c0:81:f5:25:2a:64
```




