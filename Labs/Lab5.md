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

The plan is to have the robot run for some set amount of time and shut off after that time has passed (in this case 5 seconds). I implemented three new commands over BLE: hard stop, get data, and pid tuning. Triggering either a hard stop or timing out the loop will stop the motors and then send over the sensor and timing data as well as the PID tuning parameters which have been stored on the Artemis as the robot ran.


## labtasks

### Position Control 

I started out with just proportional control initially to get an idea of what is physically reasonable for the car.

High proportional gain values will cause a lot of oscillation because it causes the car to adjust at high speeds which means the controller will constantly be adjusting to large shifts back and forth and create oscillation. Low proportional gain values are technically safer but that would slow down the system substantially to the point where the robot won't be moving fast or interestingly enough to do the tricks we require later. 

If the robot starts at 3m away from the wall, the initial error is 2700 mm. That is going to saturate immediately unless Kp is small because the upper limit on u is 255 (max PWM). So a good starting point is 0.05-0.15 as that'll get us in the range of the higher speeds of the robot. 

I also set 

```C++
distanceSensor.setTimingBudgetInMs(20)
```

to 20Hz. This means that the data update rate is much faster so the sensor is more responsive which is ideal at high speeds. At lower data update speeds the car can travel quite a lot between each reading so it causes oscillation as the car has to adapt to the latest data after it's already moved a lot. The tradeoff on this is noisier data but we're making the car stop at a relatively far enough point from the wall (304 mm) that losing accuracy by a few millimeters is preferrable over jerky car movement. I'm using the long distance mode because I wanted to make sure it had the ability to detect the wall if I placed it far away and my general logic for this lab is to trade off accuracy for more data, regardless of quality. 

When I tested closer to the wall I ended up in the deadband of the motor because my error was smaller so my commanded speed was smaller. My car's deadband is around 80. So, at the bottom of my PWM calculation code I added deadband compensation to account for it:

```C++
if (abs(u) < 80 && abs(u) > 0)
    u = 85 * sign(u);
```

This forces the car to at least drive above 85 for fine adjustment. 

The only issue with the current setup is that Kp causes overshoot which contributes heavily to my oscillation problem. To mitigate this I added some derivative control. I don't see the need for integral control at least at this point because integral control is quite prone to overshoot (wind-up issues) so it would just worsen my problems. Integral control is good for mitigating steady state error and maintaining position under constant disturbances. However, this system doesn't have a lot of disturbances so once it gets to 304 mm away from the wall, it's going to stay there because I'm driving on flat ground and no one is pushing it. 

I started with a small amount of Kd (0.02) and then adjusted it based off of how my car performed. 

Tragically my motor driver is burnt out so I have no pictures/videos at the moment. I will replace it during lab and update in the evening. 

### Relevant Code:

#### Code for handling PID command over Bluetooth
```C++
void handlePID() {
  Serial.println("Starting PID");
  prev_time = millis();
  integral = 0;
  prev_error = 0;
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {

    if (distanceSensor.checkForDataReady()) {
      float distance = distanceSensor.getDistance();
      Serial.print("distance: ");
      Serial.println(distance);
      distanceSensor.clearInterrupt();  
      float u = computePID(distance);
      
      if (u > 255) u = 150;
      if (u < -255) u = -150;
      if (u > 0) {
        Serial.print("u: ");
        Serial.println(u);
        analogWrite(LM_F, 0);
        analogWrite(RM_F, u);
        analogWrite(LM_B, 0);
        analogWrite(RM_B, 0);
      } else {
        analogWrite(LM_F, 0);
        analogWrite(RM_F, 0);
        analogWrite(LM_B, -u);
        analogWrite(RM_B, -u);
      }
      if (log_index < MAX_SAMPLES) {
        dist_log[log_index] = distance;
        time_log[log_index] = millis() - startTime;
        u_log[log_index] = u;
        log_index++;
      }
    }
  }
  analogWrite(LM_F, 0);
  analogWrite(LM_B, 0);
  analogWrite(RM_F, 0);
  analogWrite(RM_B, 0);
  stop();
  Serial.println("PID finished");
}
```

#### PID math
```C++
float computePID(float measurement) {
  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0; 
  prev_time = current_time;
  float error = measurement - setpoint;
  integral += error * dt;
  if (integral > 500) integral = 500;
  if (integral < -500) integral = -500;
  float derivative = 0;
  if (dt > 0) {
    derivative = (error - prev_error) / dt;
  }
  prev_error = error;
  float u = Kp*error + Ki*integral + Kd*derivative;
  if (abs(u) > 0 && abs(u) < 80) {
     u = 85 * (u/abs(u));
  }
  return u;   
}
```

#### Code for handling hard stops over Bluetooth
```C++
void stop() {
  analogWrite(LM_F, 0);
  analogWrite(LM_B, 0);
  analogWrite(RM_F, 0);
  analogWrite(RM_B, 0);
}
```

#### Code for sending over data logs over Blueooth
```C++
void getData() {
  for (int i = 0; i < log_index; i++) {
    tx_estring_value.clear();
    tx_estring_value.append(time_log[i]);
    tx_estring_value.append(",");
    tx_estring_value.append(dist_log[i]);
    tx_estring_value.append(",");
    tx_estring_value.append(u_log[i]);

    tx_characteristic_string.writeValue(tx_estring_value.c_str());
  }
}
```


#### Python commands
```python
time_vals = []
dist_vals = []
u_vals = []

def notification_handler(uuid, data):
    msg = data.decode()
    t, d, u = msg.split(',')
    time_vals.append(float(t))
    dist_vals.append(float(d))
    u_vals.append(float(u))
    print("RX:", msg)

    

    
time.sleep(0.2)
ble.start_notify(ble.uuid['RX_STRING'], notification_handler)
ble.send_command(CMD.HARD_STOP, "")
print("executed hard stop")
time.sleep(5)
#ble.stop_notify(ble.uuid['RX_STRING'])
ble.send_command(CMD.PID, "")
print("executed PID control")
time.sleep(10);
ble.send_command(CMD.GET_TOF, "")
print("got time of flight data")
    

plt.figure()
plt.plot(time_vals, dist_vals)
plt.xlabel("Time (ms)")
plt.ylabel("Distance (mm)")
plt.title("PID Position Control")
plt.show()


time.sleep(10)  
```