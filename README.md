# owoTrackVR and moveTrackVR differences
Instead of using the (Game) Rotation Vector, moveTrackVR will use uncalibrated IMU data, bypassing any phone automated calibration algorithms. 
By bypassing the phones autoamted calibration algorythms - which differs from phone to phone - and using a static calibration routine instead makes phones orientation data more relaiable.
This however will require additional calibration steps and filters in the Server.

# moveTrackVR Android Application
This application can be installed on an Android device to connect to [PSMoveServiceEx - Virtual Device Manager](https://github.com/Timocop/PSMoveServiceEx-Virtual-Device-Manager) to replace devices orientation data.

## Device Compatibility
All phones with gyroscope, accelerometer, and - optionally - magnetometer.

This app may also be compatible with some Wear OS devices.

## Building
The application can be easily built with Android Studio by opening the project in Android Studio.
