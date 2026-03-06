---
layout: default
title: "Lab 5"
permalink: /LAB-5/
description: "Writeup for Lab 5."
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
* [IMU](#imu)

---

## prelab 

### Debugging

The plan is to have the robot run for some set amount of time and shut off after that time has passed. In addition, if I need to stop the robot at any time I can send an 'S' as a serial input to stop it immediately. Once either of these stops are triggered, the arrays of sensor data and the PID tuning parameters are sent via bluetooth to then look at in python. 

