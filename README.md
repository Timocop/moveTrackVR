# owoTrackVR and moveTrackVR differences
Instead of using the (Game) Rotation Vector like owoTrack does, moveTrackVR will use calibrated sensor data, bypassing any phone automated calibration algorithms. 
Bypassing the phone's automated calibration algorithms - which differ from phone to phone - and instead using a Madgwick filter makes phone orientation data more reliable.

# moveTrackVR Android Application
This application can be installed on an Android device to connect to [SlimeVR server](https://github.com/SlimeVR/SlimeVR-Server) or [PSMoveServiceEx - Virtual Device Manager](https://github.com/Timocop/PSMoveServiceEx-Virtual-Device-Manager) to replace devices orientation data.

## Device Compatibility
All phones with gyroscope, accelerometer, and - optionally - magnetometer.

This app may also be compatible with some Wear OS devices.

## Building
The application can be easily built with Android Studio by opening the project in Android Studio.
