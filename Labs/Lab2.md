---
layout: default
title: "Lab 2"
permalink: /LAB-2/
description: "Writeup for Lab 2."
---

[← Back to Home]({{ '/' | relative_url }})

## Contents
* [Lab Tasks](#labtasks)
* [Accelerometer](#accelerometer)
* [Gyroscope](#gyroscope)
* [Sample Data](#sampledata)
* [Stunt](#stunt)

---

## labtasks

#### Setup IMU

![IMU Setup](../Images/IMU_setup.jpg)

The IMU plugs into the qwiic connect port on the side of the Artemis Nano. The red light is on indicating that it is receiving power. 


![IMU Data Stream](../Images/IMU_basic_stream.gif)
I started off running the basic code provided in the IMU library and rotated by IMU in random directions to see how the data values change. 

#### AD0_VAL

We're using the [ICM-20948](https://cdn.sparkfun.com/assets/7/f/e/c/d/DS-000189-ICM-20948-v1.3.pdf) as our IMU sensor for this lab. We're using I2C for communication in this instance so the AD0 pin is related to the I2C address of the board. There are two potential addresses, high and low corresponding to 1 and 0 respectively. Because two addresses are available, it is possible to connect two ICM-20948 sensors to the same I²C bus as long as each one has a different AD0 configuration. The default for this board and the example code is HIGH or AD0 = 1 so we stick with that in order for the Artemis Nano and the IMU to communicate. If we wanted to use LOW or AD0 = 0, we would need to close the ADR jumper. There's no point in doing this unless we're connecting another sensor to the second port (same I2C bus though). 

## accelerometer 

### Accelerometer

The accelerometer measures linear acceleration in m/s^2. It measures all acceleration felt in the environment (because it's based on a spring force balance) including gravity. If the accelerometer is held parallel to the ground, the acceleration value read would equal 9.81 m/s^2 (acceleration due to gravity). When tilted, the acceleration due to gravity is decomposed into the directions of the body axis of the accelerometer. Based on the values recorded in each axis we can determine the angle at which they've been rotated by. However, because accelerometer values are based on gravity, we cannot determine the rotation in yaw because the accelerometer still experiences the same acceleration in the z-direction as it remains parallel to the ground. 

For example,
$$
\a_z = g*\cos\ {\theta}
$$

$$
\a_x = g*\sin\ {\theta}
$$

$$
\theta = \arctan\frac{a_x}{a_z}
$$

$$
\phi = \arctan\frac{a_y}{z_z}
$$

#### Code

I took the accelerometer data and sent it via BLE to my computer where I graphed the values in python. The data on the Arduino was combined into a large string with the time stamps, accelerometer data in every direction, and gyroscope data in every direction separated by commas. I then processed the data inside the notification handler in python by parsing using the commas and then sorting each into separate arrays. 

![Raw Accel Data](../Images/raw_accel_data.png)

I was moving the IMU around while taking the data so you can see a lot of peaks and noise from the shaking. 

```python
    def notification_handler(uuid, data):
        print("notification handler")
        msg = data.decode()
        times.append(float(msg.split(",")[0]))
        accX.append(float(msg.split(",")[1]))
        accY.append(float(msg.split(",")[2]))
        accZ.append(float(msg.split(",")[3]))
        gyrX.append(float(msg.split(",")[4]))
        gyrY.append(float(msg.split(",")[5]))
        gyrZ.append(float(msg.split(",")[6]))
        

    times.clear()
    accX.clear()
    accY.clear()
    accZ.clear()
    gyrX.clear()
    gyrY.clear()
    gyrZ.clear()  
    ble.start_notify(ble.uuid['RX_STRING'], notification_handler)


    ble.send_command(CMD.GET_IMU, "")

    time.sleep(2)  

    ble.stop_notify(ble.uuid['RX_STRING'])
```

Code for graphing the IMU raw data:

```python
    def init_plots():
        plt.plot(times, accX, label="Accelerometer X")
        plt.plot(times, accY, label="Accelerometer Y")
        plt.plot(times, accZ, label="Accelerometer Z")
        
        plt.xlabel("Time (s)")
        plt.ylabel("Accelerometer")
        plt.title("Accelerometer Data vs Time")
        plt.legend()
        plt.grid(True)
        
        plt.show()

        plt.plot(times, gyrX, label="Gyroscope X")
        plt.plot(times, gyrY, label="Gyroscope Y")
        plt.plot(times, gyrZ, label="Gyroscope Z")
        
        plt.xlabel("Time (s)")
        plt.ylabel("Gyroscope")
        plt.title("Gyroscope Data vs Time")
        plt.legend()
        plt.grid(True)

        plt.show()
```

The accelerometer data is fairly accurate, if not slightly noisy. Here's the data when the accelerometer is still (but not in any particular orientation)

![Raw Accel Data](../Images/steady_raw_accel_data.png)

![Raw Accel Data](../Images/angle_data_accel.png)

I tried to get the accelerometer to be at -90,0,90. I think there is a constant offset in the accelerometer because it's consistently 114 deg at physically 90 deg angles. So, with that offset in mind this is what the serial monitor looked like. 

![Raw Accel Data](../Images/accel_config.png)

#### Fourier Transform

I transformed the data to see where the highest amplitude frequencies are to determine where to put the cutoff frequency for a low pass filter. Below is the transform code for the raw acceleration data in x (similar code for y and z) as well as the transform code for theta and phi.

```python
    dt = np.mean(np.diff(times)) / 1000
    Fs = 1 / dt

    N = len(accX)

    fft_vals_x = np.fft.fft(accX)
    freqs_x = np.fft.fftfreq(N, d=dt)

    positive_x = freqs_x > 0

    freqs_x = freqs_x[positive_x]
    fft_magnitude_x = np.abs(fft_vals_x[positive_x])

    plt.plot(freqs_x, fft_magnitude_x, label="FFT of Accelerometer X")
```

For theta
```python
    fft_vals_th = np.fft.fft(theta)
    freqs_th = np.fft.fftfreq(N, d=dt)

    positive_th = freqs_th > 0

    freqs_th = freqs_th[positive_th]
    fft_magnitude_th = np.abs(fft_vals_th[positive_th])

    plt.plot(freqs_th, fft_magnitude_th, label="FFT of Theta")
```

For phi
```python
    fft_vals_phi = np.fft.fft(phi)
    freqs_phi = np.fft.fftfreq(N, d=dt)

    positive_phi = freqs_phi > 0

    freqs_phi = freqs_phi[positive_phi]
    fft_magnitude_phi = np.abs(fft_vals_phi[positive_phi])

    plt.plot(freqs_phi, fft_magnitude_phi, label="FFT of Phi")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.title("FFT of Angles")
    plt.grid(True)
    plt.show()
```

![alt text](...\Images\FFT_accel_data.png)

![alt text](...\Images\FFT_angles.png)

If table vibration is introduced:

![alt text](...\Images\table_vibration_data.png)

![alt text](...\Images\table_vibration_FFT.png)

#### Low Pass Filtering

Based on this data, it looks like a good cutoff frequency would be around 10-12 because there's a major peak after then. This makes sense to me because most of the signal of interest (tilt or orientation changes) occurs at low frequencies, typically below 5–10 Hz, since human or board movements are slow. Anything at much higher frequencies is most likely electrical noise or mechanical vibration. Choosing the correct cutoff frequency is important because if the cutoff is too conservative, you risk missing actual useful data. If the cutoff is too high, you introduce excessive noise into your sensor readings. 

With a cutoff frequency of 10 Hz, the comparison between the angle data before and after filtering is shown below. The low pass filtered data has less excessive spikes than the raw data which is exactly the behavior expected. There is probably a low pass filter built into the board because I don't see excessively high frequencies meaning a lot of the electrical noise is probably already filtered out. When I tried vibrating the table by lightly tapping near the IMU, the FFT didn't have giant spikes near the higher frequencies which indicates to me that it's been filtered out already. 

![alt text](...\Images\filtered_vs_raw.png)

The forumlas for a low pass filter are \theta_{\text{LPF}}[n] = \alpha \cdot \theta_{\text{RAW}} + (1-\alpha) \cdot \theta_{\text{LPF}}[n-1] \\
\theta_{\text{LPF}}[n-1] = \theta_{\text{LPF}}[n]

The code for the filtered graph:
```python
    fc = 10 

    alpha = (2*np.pi*fc*dt) / (1 + 2*np.pi*fc*dt)

    filtered = np.zeros_like(theta)
    filtered[0] = theta[0]

    for i in range(1, len(theta)):
        filtered[i] = alpha*theta[i] + (1-alpha)*filtered[i-1]

    plt.plot(times, theta, label="Raw")
    plt.plot(times, filtered, label="Filtered")
    plt.legend()
    plt.show()
```

Sidenote: When working with angles and converting between radians and degrees and in general doing trig, it's important to use atan2 in order to utilize the entire -\pi to \pi range. I also coded a wrap-around function to make sure that the angle values remain within that range:

```python
    def wrap_angle(angle):
        return (angle + np.pi) % (2*np.pi) - np.pi
```

## gyroscope



