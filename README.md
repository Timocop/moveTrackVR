![moveTrackLogo](https://github.com/Timocop/moveTrackVR/assets/22834512/b2c49134-e064-408c-800b-4175f71689d9)

# owoTrackVR and moveTrackVR differences
**moveTrackVR** utilizes un/calibrated sensor data instead of the (Game) Rotation Vector. This approach bypasses any automated calibration algorithms implemented by the phone. Many phones incorporate customized sensor algorithms, including drift correction and custom dead zones, which vary across different devices. Unfortunately, the (Game) Rotation Vector on most phones tends to be unreliable due to these customized algorithms.

**moveTrackVR** is compatible with all **owoTrackVR** servers!

![image](https://github.com/Timocop/moveTrackVR/assets/22834512/666156d0-fbd3-49e3-a637-892e87fd83ef)

## 🎥 Video Demonstration with a Fairphone - First Edition
[ExMoveTrack2.webm](https://github.com/Timocop/moveTrackVR/assets/22834512/5a5642d6-e2bc-4e15-8ace-cd44cdaf94ef)


# moveTrackVR Android Application
This application can be installed on an Android device to connect to [SlimeVR server](https://github.com/SlimeVR/SlimeVR-Server) or [PSMoveServiceEx - Virtual Device Manager](https://github.com/Timocop/PSMoveServiceEx-Virtual-Device-Manager) to replace devices orientation data.

## Device Compatibility
All phones with gyroscope, accelerometer, and - optionally - magnetometer.

This app may also be compatible with some Wear OS devices.

## Building
The application can be easily built with Android Studio by opening the project in Android Studio.
