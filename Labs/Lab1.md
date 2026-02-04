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

Blink
![Blinking LED](../Images/blink.gif)

Serial
![Serial Monitor](../Images/serial_example.png)

Analog Read
![Analog Read Temperature](../Images/temp_example.png)

Microphone Output
![Microphone Output](../Images/mic_example.png)

Arduino:
My Arduino IDE was already setup so I just had to download the packages specific for this board. For the example code I changed the baud rate to match up with the preexisting baud rate of my COM ports (9600). 


Lab 1B:
To setup the Bluetooth connection, I installed the ArduinoBLE library, opened up ble_arduino from the unzipped files, and burned the code. The MAC address was then printed to the serial monitor and I updated connection.yaml with the MAC address and the generated UUID. 

For the most part, I followed the instructions in Lab 1 exactly. In order to activate my virtual environment for this class however, I found that I had to use
```bash
    Set-ExecutionPolicy Unrestricted -Scope Process
```
in order to give it the permissions to run.

MAC Address (Did not put the whole thing because of privacy concerns)
```bash
    Advertising BLE with MAC: c0:81:...
```

UIUD Updated in connection.yaml

```python
ble_service: 'b831d150-5b5d-4069-ac6c-820891ea74cf'

characteristics:
  TX_CMD_STRING: '9750f60b-9c9c-4158-b620-02ec9521cd99'

  RX_FLOAT: '27616294-3063-4ecc-b60b-3470ddef2938'
  RX_STRING: 'f235a225-6735-4d73-94cb-ee5dfce9ba83'
```

Codebase
The Artemis board receives commands as strings with a TX characteristic (transmit). Through switch-case logic, it takes in a command specified through that transmission and triggers some behavior (returns time stamps, echos, etc.) These commands can be parsed using methods from RobotCommand and EString. 

UIUDs are universally unique identifiers and they're important for differentiating the different data types sent and received. There are different UIUDs for different characteristics, in this case TX strings, RX floats, RX strings. 

When data is sent from the the Artemis (via the arduino code) to the computer (via the python code) it is notified via the BLE (Bluetooth Low Energy) notifications. You can have different types of characteristics, in this instance I only used float and string characteristics. 

These characteristics determine the type of data received by the computer. These notifications allow Python to asynchronously receive the data packets and then store them into Python lists to then analyze on the computer. It's asynchronous because the computer receives data only when it is written rather than at some set time interval. 


Lab Tasks 



