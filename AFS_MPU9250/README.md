# AFS_MPU9250

## Introduction

This is an Arduino library for a MPU-9250 Motion Processing Unit (or IMU). It's sensor components been constructed as an Adatafruit_Sensor (AFS) objects. It bears a striking resemblance to the Adafruit_MPU6050 library, but includes some mods for accessing the AK8963 Magnetometer device and data, and setting up the data ready interrupt.

## Warning

This is the first publication of this library.

Though it's been used within an application, it will certainly need refinement. Some of the default configuration items are a little too bolted down and biased toward my application, and need to be more open to other configurations.