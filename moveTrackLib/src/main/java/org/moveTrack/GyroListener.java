package org.moveTrack;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;

import org.moveTrack.math.MadgwickAHRS;

public class GyroListener implements SensorEventListener {
    private static final float NS2S = 1.0f / 1000000000.0f;

    private static final long MADGWICK_UPDATE_RATE_MS = 5;

    private SensorManager sensorManager;

    private Sensor GyroSensor;
    private Sensor AccelSensor;
    private Sensor MagSensor;


    private float[] rotation_quat;
    private float[] gyro_vec;
    private float[] accel_vec;
    private float[] mag_vec;

    private long last_madgwick_timestamp;
    private long last_gyro_timestamp;

    private long elapsed_gyro_time;

    private int ROTATION_SENSOR_TYPE;

    private String sensor_type = "";

    private MadgwickAHRS filter_madgwick;

    UDPGyroProviderClient udpClient;

    GyroListener(SensorManager manager, UDPGyroProviderClient udpClient_v, AppStatus logger, AutoDiscoverer.ConfigSettings configSettings) throws Exception {
        sensorManager = manager;

        GyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        if(GyroSensor == null) {
            GyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
            if(GyroSensor == null) {
                logger.update("Gyroscope sensor could not be found, this data will be unavailable.");
            }
            else {
                logger.update("Uncalibrated gyroscope sensor found! Might cause sensor drift!");
            }
        }
        else {
            logger.update("Gyroscope sensor found!");
        }

        AccelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if(AccelSensor == null) {
            AccelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED);
            if(AccelSensor == null) {
                logger.update("Accelerometer sensor could not be found, this data will be unavailable.");
            }
            else {
                logger.update("Uncalibrated accelerometer sensor found! Might cause sensor drift!");
            }
        }
        else {
            logger.update("Accelerometer sensor found!");
        }

        if(configSettings.magnetometerEnabled)
        {
            MagSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
            if(MagSensor == null) {
                logger.update("Magnetometer sensor could not be found, this data will be unavailable.");
            }
            else {
                logger.update("Magnetometer sensor found!");
            }
        }
        else {
            logger.update("Magnetometer disabled by user!");
        }

        float madgwickBeta = configSettings.madgwickBeta;
        if (madgwickBeta < 0.0f)
            madgwickBeta = 0.0f;
        if (madgwickBeta > 1.0f)
            madgwickBeta = 1.0f;

        rotation_quat = new float[4];
        gyro_vec = new float[3];
        accel_vec = new float[3];
        mag_vec = new float[3];
        last_gyro_timestamp = 0;
        last_madgwick_timestamp = 0;
        elapsed_gyro_time = 0;
        filter_madgwick = new MadgwickAHRS(0.0f, madgwickBeta);

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
        if(event.timestamp == 0)
            return;

        if(event.sensor.getType() == Sensor.TYPE_GYROSCOPE){
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            gyro_vec = vec;

            udpClient.provide_gyro(event.timestamp, vec);

            if(last_gyro_timestamp != 0)  {
                final float lastGyro = (event.timestamp - last_gyro_timestamp) * NS2S;
                elapsed_gyro_time += (long)(lastGyro * 1000);

                if(elapsed_gyro_time > MADGWICK_UPDATE_RATE_MS) {
                    updateMadgwick(event.timestamp);
                    elapsed_gyro_time = 0;
                }
            }

            last_gyro_timestamp = event.timestamp;
        }
        else if(event.sensor.getType() == Sensor.TYPE_GYROSCOPE_UNCALIBRATED){
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            gyro_vec = vec;

            udpClient.provide_uncalib_gyro(event.timestamp, vec);

            if(last_gyro_timestamp != 0)  {
                final float lastGyro = (event.timestamp - last_gyro_timestamp) * NS2S;
                elapsed_gyro_time += (long)(lastGyro * 1000);

                if(elapsed_gyro_time > MADGWICK_UPDATE_RATE_MS) {
                    updateMadgwick(event.timestamp);
                    elapsed_gyro_time = 0;
                }
            }

            last_gyro_timestamp = event.timestamp;
        }
        else if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER){
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            accel_vec = vec;

            udpClient.provide_accel(event.timestamp, vec);
        }
        else if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER_UNCALIBRATED){
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            accel_vec = vec;

            udpClient.provide_uncalib_accel(event.timestamp, vec);
        }
        else if(event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD){
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            mag_vec = vec;

            udpClient.provide_mag(event.timestamp, vec);
        }
        else if(event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED){
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            mag_vec = vec;

            udpClient.provide_uncalib_mag(event.timestamp, vec);
        }
    }

    private void updateMadgwick(long timeStamp)
    {
        if(last_madgwick_timestamp != 0) {
            final float deltaTime = (timeStamp - last_madgwick_timestamp) * NS2S;

            filter_madgwick.setSamplePeriod(deltaTime);

            if(MagSensor != null) {
                filter_madgwick.update(
                        gyro_vec[0], gyro_vec[1], gyro_vec[2],
                        accel_vec[0], accel_vec[1], accel_vec[2],
                        mag_vec[0], mag_vec[1], mag_vec[2]);
            }
            else {
                filter_madgwick.update(
                        gyro_vec[0], gyro_vec[1], gyro_vec[2],
                        accel_vec[0], accel_vec[1], accel_vec[2]);
            }

            float[] quat = filter_madgwick.getQuaternion();
            float[] swapQuat = new float[4];
            swapQuat[0] = quat[1];
            swapQuat[1] = quat[2];
            swapQuat[2] = quat[3];
            swapQuat[3] = quat[0];

            udpClient.provide_rot(timeStamp, swapQuat);
        }

        last_madgwick_timestamp = timeStamp;
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
