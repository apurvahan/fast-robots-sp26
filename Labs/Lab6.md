---
layout: default
title: "Lab 6"
permalink: /LAB-6/
description: "Writeup for Lab 6."
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
* [Timing](#timing)

---

# prelab 
The overall structure of the code remains the same from last lab, except with orientation PID values instead of distance PID values. I use the SEND_THREE_FLOATS command to send 4 floats over bluetooth: Kp, Ki, Kd, and the orientation heading desired (currently 20 degrees). 

Arduino side:
```C++
case SEND_THREE_FLOATS:
  float fl_a, fl_b, fl_c, fl_d;
  robot_cmd.get_next_value(fl_a);  
  robot_cmd.get_next_value(fl_b);
  robot_cmd.get_next_value(fl_c);
  robot_cmd.get_next_value(fl_d);
  Kp = fl_a;
  Ki = fl_b;
  Kd = fl_c;
  setpoint = fl_d;
  break;
```
Python side:
```python
time.sleep(0.2)
ble.start_notify(ble.uuid['RX_STRING'], notification_handler)
ble.send_command(CMD.HARD_STOP, "")
time.sleep(5)
ble.send_command(CMD.SEND_THREE_FLOATS, "0.5|0|1.5|20") #pid 
time.sleep(2)
ble.send_command(CMD.PID_ORIENTATION, "")
print("executed PID control")
time.sleep(30)
ble.send_command(CMD.GET_IMU, "")
print("got imu data")

time.sleep(30)
    
ble.stop_notify(ble.uuid['RX_STRING'])


plt.figure()
plt.plot(time_vals, ang_vals)
plt.xlabel("Time (ms)")
plt.ylabel("Angle (deg)")
plt.title("PID Angle Control")
plt.show()


time.sleep(10)  
```

# labtasks

## DMP

Using the tutorial linked in the lab. Line 29 in the general library file did not include that constant and in general had very minimal references to DMP. I looked through the library some more and it didn't seem like it was referenced in the same way everywhere. I ended up going to the github documentation for the board (https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/DMP.md) and saw that the last version that they mention in the DMP section is 1.2.X so I downgraded my IMU library installation to 1.2.13 and I was able to uncomment the USE_DMP constant. It's definitely possible I missed something but I'm wondering if they changed the way of doing DMP in the newer library version.

Setup code for DMP, referencing Stephan Wagner's tutorial 
```C++
myICM.begin(Wire, 1);
  if (myICM.status != ICM_20948_Stat_Ok) {
    Serial.println("IMU failed to boot!");
    while(1);
  }
  Serial.println("IMU OK");


  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  Serial.println("initialized DMP");
  Serial.println(success);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  Serial.println("enable DMP");
  Serial.println(success);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // FIX: was Quat6
  Serial.println("set DMPOD rate");
  Serial.println(success);
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  Serial.println("enable FIFO");
  Serial.println(success);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  Serial.println("enable DMP");
  Serial.println(success);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  Serial.println("reset DMP");
  Serial.println(success);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  Serial.println("reset FIFO");
  Serial.println(success);

  if (!success) {
      Serial.println("DMP initialisation failed!");
      while (1);
    }
```

### DMP Implementation 
I start my loop by checking the DMP and getting the yaw value at the start. That yaw value defines my offset for that run as I care about the relative yaw for this system which is yaw - yaw_offset. Every PID look I check the DMP.

The DMP uses quaternions instead of Euler angles. Quaternions are generally better than euler angles because they avoid gimball lock but that doesn't particularly matter in this case because we only care about motion in the yaw direction anyways. However, the DMP data is in terms of quaternions so I use it here to calculate the angle. 

$$
\psi_{\text{yaw}} = \text{atan2}\!\Big(2(q_0 q_3 + q_1 q_2),\ 1 - 2(q_2^2 + q_3^2)\Big)
$$

$$
q_0 = \sqrt{\max\!\left(0,\ 1 - q_1^2 - q_2^2 - q_3^2\right)}
$$


### Quat9 vs Quat6

The example given to us used Quat6 but I tried to use Quat9 because I think it will give me better results because its fusing data from 9 sources (3 accelerometer directions, 3 gyroscope directions, and 3 magnetometer directions). The gyroscope data tracks the rotation and the acceleration uses gravity as a reference for pitch and roll (not super useful in this case as we only care about yaw). The magnetometer tries to fix yaw to north and uses the magnetic field to do so so while it's good for mitigating gyro drift, it can be very inaccurate when around things that emit a lot of EMI (like motors and wires). I chose to use Quat9 to avoid yaw drift and having to track and correct for it over time. However on reflection, I think the magnetometer data is quite inaccurate, especially because I was doing my testing in Upson, surrounded by devices and the electronics of the robot itself. So, something I will test in the future is switching back to Quat6 to see if that improves my data. 

```C++
bool check_DMP() {
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || 
      (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {

      if ((data.header & DMP_header_bitmap_Quat9) > 0) {
          // Convert fixed-point to float (scale factor is 2^30)
        double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
        double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
        double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
        double q0 = sqrt(max(0.0, 1.0 - q1*q1 - q2*q2 - q3*q3)); 
        double yaw_rad = atan2(2.0*(q0*q3 + q1*q2),
                           1.0 - 2.0*(q2*q2 + q3*q3));
        yaw = (float)(yaw_rad * (180.0 / PI));  // store in degrees

        
        return true;

      } else {
        return false;
      }
  } else {
    return false;
  }
}
```

### Filtering 

My robot was oscillating quite a lot while trying to maintain the angle. After tuning the PID controls for a while I realized that the issue might be jerky data values. A low pass filter helps with that because it blocks out sources of noise that could be causing the sensor to overshoot and overcorrect each time.

```C++
yfiltered_d = (1 - df_alpha) * yfiltered_d + (df_alpha) * gyrZ_now;  
```

This ended up helping a decent amount but I still had some issue with oscillation except it was happening within a narrower range of values. The degrees it was oscillating at were negligibly small enough that I could have my error target a narrow range of angles rather than an exact value.

```C++
float error = setpoint - measured_yaw;
  if (abs(error) <= 2) {
    return 0;
  }
```

If my u value is 0, it drives the motors at 0 PWM so it's a complete stop. 

### Bias

The other thing I do to mitigate error at startup is to take into account the bias that the sensor has. To do so I get the average over 200 cycles of the gyro values while the car is still to see on average if the data drifts in a particular direction. I then compensate for that shift every time I calculate a new gyroscope value in my main PID method. 

```C++
void calibrateGyroBias() {
  float sum = 0;
  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    myICM.getAGMT();
    sum += myICM.gyrZ();
    delay(5);
  }
  gyrZ_bias = sum / samples;
}
```

## PID Tuning

### Static Friction Compensation

It turns out that the speed at which I need to run my motors to get it to noticably spin versus the speed at which I need to run my motors at to get them to spin forward are different. It's most likely because the wheels are encountering more friction when they turn because it's not in the direction of the rolling part of the wheel (moment of inertia). My minimum speed ended up being around a PWM of 75 for turning. I changed my implementation of this from Lab 5 where I map the u value as a percentage of 255 and then map it onto the range (75,255). The previous way I did it, where if u dropped below a specified threshhold it would just spin at that threshhold, caused issues with getting the car to slow down as it approached the required angle. 

```C++
const float MIN_SPEED = 75.0;
    if (abs(u) > 0 && abs(u) < MIN_SPEED) {
      scaled_u = (1-(abs(u)/255))*(255-MIN_SPEED) + MIN_SPEED;
      u = scaled_u * (u > 0 ? 1.0 : -1.0);
    }
    u = constrain(u, -255, 255);
```

### PID Math

I have a method called handleOrientationPID() that's called for the robot to start moving. It resets the FIFO queue for the DMP so that there aren't accumulated data values on the data stack from the last run. I also start off with a yaw offset as I mentioned above and correct for it every time I get the yaw value. I run my loop for 30 seconds and each run I check the DMP for new yaw values and then computer PID using the relative yaw and the current gyroscope value (minus the bias). In my PID method I check if the error is too small (close enough to the target angle that I can stop calculating) and if not I get my u value using the PID equation:

$$
u = K_p \cdot e(t) + K_i \cdot \int e(t)\, dt + K_d \cdot \frac{de(t)}{dt}
$$

Proportional controls how aggressively the robot reacts to the current error, so if the Kp value is too low the robot is quite sluggish and slow to response but if it's too fast the robot will oscillate around the target because it'll cycle between over and under shooting. 

Integral corrects for steady-state error because it integrates that error over time. While that is helpful for mitigating steady state error that proportional control is generally not good at solving, integral control can create a lot of oscillation from a cycle of integrating the small over and undershoot. 

Derivative is good as a damper because the magnitude is based on how fast the error is changing which is useful for decreasing overshoot. However, if the data is already noisy (which is probable in cases like sensor data), the derivatives aren't stable so the overall response is quite jittery. 

I started off with proportional control until I got the oscillation down as much as I could and then I added integral control and derivative control in separately (so tried PI and then PD). Integral control worsened my issues as expected because the oscillations just got more aggressive. In the end I ended up with no integral control because my issue wasn't reaching the angle (no steady state error in my system) and integral control can't help with that problem. I started off with high (~5) Kd values before it started oscillating a lot and I started decreasing Kd until the robot turned more reasonably. In the end I ended up with a Kp = 0.05 and a Kd = 0.02 which are quite small but reasonable given the oscillation. 

I used the gyroscope data directly for proportional control because it's already in terms of angular velocity so there's no additional math needed for it. 

While I was testing for integral control, to prevent it from getting out of control I had upper and lower bounds that prevented u from going above 255. 


### PID Code

```C++
float computeOrientationPID(float measured_yaw, float gyrZ_now) {
  unsigned long now = millis();
  float dt = (float)(now - prev_time) / 1000.0f;
  float yfiltered_d = 0;
  float df_alpha = 0.8;
  prev_time = now;
  if (dt <= 0) dt = 0.001f;  
  //measured_yaw = measured_yaw + gyrZ_now*dt;
  //yaw = measured_yaw;

  float error = setpoint - measured_yaw;
  if (abs(error) <= 2) {
    return 0;
  }
  //while (error >  180.0f) error -= 360.0f;
  //while (error < -180.0f) error += 360.0f;

  integral += error * dt;
  integral  = constrain(integral, -200.0f, 200.0f);

  yfiltered_d = (1 - df_alpha) * yfiltered_d + (df_alpha) * gyrZ_now;  
  float u = Kp * error + Ki * integral - Kd * yfiltered_d;
  return u;
}
```

```C++
void handleOrientationPID() {
  Serial.println("Starting Orientation PID");
  yaw = 0.0;
  integral = 0;
  prev_error = 0;
  log_index = 0;
  prev_time = millis();
  unsigned long startTime = millis();
  float yaw_offset = yaw;
  myICM.resetFIFO();
  while (!check_DMP());  // wait for first real reading
  yaw_offset = yaw;
  float scaled_u = 0;

  while (millis() - startTime < 30000) {   
    //myICM.getAGMT(); 
    check_DMP();
    float relative_yaw = yaw - yaw_offset;
    float u = computeOrientationPID(relative_yaw, myICM.gyrZ() - gyrZ_bias);
    //float u = computeOrientationPID(yaw, myICM.gyrZ() - gyrZ_bias);

    const float MIN_SPEED = 75.0;
    if (abs(u) > 0 && abs(u) < MIN_SPEED) {
      scaled_u = (1-(abs(u)/255))*(255-MIN_SPEED) + MIN_SPEED;
      u = scaled_u * (u > 0 ? 1.0 : -1.0);
    }
    u = constrain(u, -255, 255);

    drive(u, 0);

    if (log_index < MAX_SAMPLES) {
      yaw_log[log_index] = yaw;
      Serial.println(yaw);
      time_log[log_index] = millis() - startTime;
      u_log[log_index] = u;
      log_index++;
    }
  }

  stop();
}

void drive(float u, float forward_speed) {
    int left  = (int)constrain(u, -255, 255);
    int right = (int)constrain(-u, -255, 255);

    analogWrite(LM_F, left  > 0 ?  left  : 0);
    analogWrite(LM_B, left  < 0 ? -left  : 0);
    analogWrite(RM_F, right > 0 ?  right : 0);
    analogWrite(RM_B, right < 0 ? -right : 0);
}
```


For driving code, I condensed the logic down significantly. The wheels spin in opposite direction and baed on whether they are commanded a positive or negative u, the right motor pin is set to high. 

The first video depicts my initial tuning. I ended up with extremely high derivative control (5) and low proportional control and it was strange because it seemed to work at some combinations of those parameters and not others. I eventually figured out that no matter how much I tweaked it my derivative control being high was responsible for a lot of the oscillation so I brought it down to a much smaller value of 1.5 which ended up being my final value. I upped my proportional control from my previous testing around 0.1 to 0.5. The second video is when I tried to mess with the integral control to see if that would help with minimizing the oscillation but I think I still had issues with overshoot, especially when it came to the first turn. I had an integral control value of around 0.2 which I ended up bringing down to around 0.02 and finally 0 because similar to Lab 5 I don't think I actually need PID for my robot. PD seems to work alright provided my derivative control isn't too high. 

By the third video I finalized my PID numbers: 0.5 for proportional and 1.5 for derivative. It worked fairly well except for the vibration that happens around the final value. This is where I implemented the low pass filter and making the target value a small range of angles instead of a fixed angle. The fourth video is with all that implemented and the car is much more stable in that one. 

[![Initial tuning:](https://youtube.com/shorts/6FWKk5--WoE)](https://youtube.com/shorts/6FWKk5--WoE)

[![Integral tuning:](https://youtube.com/shorts/vJ3IatulNMw)](https://youtube.com/shorts/vJ3IatulNMw)

[![Completed PD tuning:](https://youtube.com/shorts/Binja-x659Q)](https://youtube.com/shorts/Binja-x659Q)

[![Effects of filtering:](https://youtube.com/shorts/bpwmEIdF8CM)](https://youtube.com/shorts/bpwmEIdF8CM)


![Orientation graph](../Images/Lab5/orientation_graph.png)
This graph starts with the robot reaching steady state position and then I agitate it repeatedly by hitting it. It returns back to the original position each time. 