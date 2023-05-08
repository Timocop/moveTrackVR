package org.moveTrack;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;

public class GyroListener implements SensorEventListener {
    private SensorManager sensorManager;

    private Sensor GyroSensor;
    private Sensor AccelSensor;
    private Sensor MagSensor;


    private float[] rotation_quat;
    private float[] gyro_vec;

    private int ROTATION_SENSOR_TYPE;

    private String sensor_type = "";

    UDPGyroProviderClient udpClient;

    GyroListener(SensorManager manager, UDPGyroProviderClient udpClient_v, AppStatus logger) throws Exception {
        sensorManager = manager;

        GyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
        if(GyroSensor == null) {
            GyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
            if(GyroSensor == null) {
                logger.update("Gyroscope sensor could not be found, this data will be unavailable.");
            }
            else {
                logger.update("Uncalibrated gyroscope sensor could not be found, using calibrated gyroscope sensor instead.");
            }
        }

        AccelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED);
        if(AccelSensor == null) {
            AccelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            if(AccelSensor == null) {
                logger.update("Accelerometer sensor could not be found, this data will be unavailable.");
            }
            else {
                logger.update("Uncalibrated accelerometer sensor could not be found, using calibrated accelerometer sensor instead.");
            }
        }

        MagSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED);
        if(MagSensor == null) {
            MagSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
            if(MagSensor == null) {
                logger.update("Magnetometer sensor could not be found, this data will be unavailable.");
            }
            else {
                logger.update("Uncalibrated magnetometer sensor could not be found, using calibrated magnetometer sensor instead.");
            }
        }

        rotation_quat = new float[4];
        gyro_vec = new float[3];

        udpClient = udpClient_v;
        udpClient.set_listener(this);
    }

    private Handler mHandler;
    public void register_listeners() {
        mHandler = new Handler();
        sensorManager.registerListener(this,GyroSensor, SensorManager.SENSOR_DELAY_FASTEST, mHandler);
        sensorManager.registerListener(this,AccelSensor, SensorManager.SENSOR_DELAY_FASTEST, mHandler);
        sensorManager.registerListener(this,MagSensor, SensorManager.SENSOR_DELAY_FASTEST, mHandler);
    }

    public void stop(){
        sensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if(event.sensor.getType() == Sensor.TYPE_GYROSCOPE_UNCALIBRATED){
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            udpClient.provide_un_gyro(event.timestamp, vec);
        }else if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER_UNCALIBRATED){
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            udpClient.provide_un_accel(event.timestamp, vec);
        }else if(event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED){
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            udpClient.provide_un_mag(event.timestamp, vec);
        }else if(event.sensor.getType() == Sensor.TYPE_GYROSCOPE){
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            udpClient.provide_gyro(event.timestamp, vec);
        }else if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER){
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            udpClient.provide_accel(event.timestamp, vec);
        }else if(event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD){
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            udpClient.provide_mag(event.timestamp, vec);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        //magnetometerAccuracy = accuracy;
        //if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.KITKAT_WATCH) {
        //    System.out.println("@@@ Accuracy for " + sensor.getStringType() + " : " + String.valueOf(accuracy));
        //}

        //udpClient.provide_accuracy(accuracy);
    }
}
