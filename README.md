# ELEC0036 Sensor Fusion
ELEC0036: Sensor Fusion - Indoor Localisation via Bluetooth Low Energy and Inertial Measurement Unit Sensor Fusion, implemented on an Arduino Nano ESP32. 

## Project Overview
This project focuses on developing an Indoor Positioning System (IPS) using Bluetooth Low Energy (BLE) and Inertial Measurement Units (IMUs) to fuse sensor data and provide accurate indoor localisation. The system leverages Received Signal Strength Indicator (RSSI) trilateration from fixed Raspberry Pico transmitters and the mobile Arduino Nano ESP32 receiver. This is then combined with accelerometer and gyroscope data from the BNO085 for step detection and heading determination. The BLE and IMU technologies were combined through a Kalman Filter and a Particle Filter. The proposed solution achieves positional inference by using a Particle Filter with an average error of 0.9952 m and a standard deviation of 0.5575 m.

## Project Goals
1. Accuracy: To achieve an IPS that has greater accuracy when employing sensor
fusion as compared to individual sensor measurements
2. Robustness: To ensure reliable performance when faced with issues like multipathing and signal attenuation
3. Latency: To provide real-time positioning information with reduced delay
4. Scalability: To have a system that can be easily deployed whilst remaining economically viable during large-scale deployment
5. Power Consumption: To ensure solution can be deployed in battery-powered
applications for long durations
