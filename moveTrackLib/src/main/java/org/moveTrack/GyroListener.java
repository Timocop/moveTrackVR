package org.moveTrack;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.util.Log;

import org.moveTrack.math.MadgwickAHRS;
import org.moveTrack.math.Quaternion;
import org.moveTrack.math.Vector3;

public class GyroListener implements SensorEventListener {
    private static final float NS2S = 1.0f / 1000000000.0f;

    private static final long MADGWICK_UPDATE_RATE_MS = 5;
    private static final long MADGWICK_SMARTRESET_MS = 500;
    private static final float MADGWICK_SMARTRESET_GRAV_STABLE = 0.25f;
    private static final float MADGWICK_SMARTRESET_DEG_MIN = 5.0f;
    private static final float MADGWICK_SMARTRESET_DEG_MAX = 25.0f;
    private static final boolean MADGWICK_SMARTRESET_INSTANT = true;

    private static final float STABILIZATION_GYRO_MAX_DEG = 1.f;
    private static final float STABILIZATION_GYRO_MIN_DEG = 0.1f;

    public final static boolean OPTIMIZE_ROTATION_PACKET_SEND = true;


    UDPGyroProviderClient udpClient;
    private SensorManager sensorManager;
    private Sensor GyroSensor;
    private Sensor AccelSensor;
    private Sensor MagSensor;

    Quaternion rot_vec;
    private float[] gyro_vec;
    private float[] accel_vec;
    private float[] gav_vec;
    private float[] lin_accel_vec;
    private float[] mag_vec;

    private long last_send_timestamp;
    private long last_madgwick_timestamp;
    private long last_gyro_timestamp;
    private long elapsed_gyro_time;
    private long gyro_samples;
    private int ROTATION_SENSOR_TYPE;
    private String sensor_type = "";
    private MadgwickAHRS filter_madgwick;
    private MadgwickAHRS filter_madgwick_quick;
    private float madgwick_beta;
    private boolean madgwick_reset;

    private boolean use_stabilization;
    private boolean use_smartcorrection;
    private float smartcorrection_time;
    private int sensor_data;
    private Handler mHandler;

    GyroListener(SensorManager manager, UDPGyroProviderClient udpClient_v, AppStatus logger, AutoDiscoverer.ConfigSettings configSettings) throws Exception {
        sensorManager = manager;

        GyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        if (GyroSensor == null) {
            GyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
            if (GyroSensor == null) {
                logger.update("Gyroscope sensor could not be found, this data will be unavailable.");
            } else {
                logger.update("Uncalibrated gyroscope sensor found! Might cause sensor drift!");
            }
        } else {
            logger.update("Gyroscope sensor found!");
        }

        AccelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if (AccelSensor == null) {
            AccelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED);
            if (AccelSensor == null) {
                logger.update("Accelerometer sensor could not be found, this data will be unavailable.");
            } else {
                logger.update("Uncalibrated accelerometer sensor found! Might cause sensor drift!");
            }
        } else {
            logger.update("Accelerometer sensor found!");
        }

        if (configSettings.magnetometerEnabled) {
            MagSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
            if (MagSensor == null) {
                logger.update("Magnetometer sensor could not be found, this data will be unavailable.");
            } else {
                logger.update("Magnetometer sensor found!");
            }
        } else {
            logger.update("Magnetometer disabled by user!");
        }

        madgwick_beta = configSettings.madgwickBeta;
        if (madgwick_beta < 0.0f)
            madgwick_beta = 0.0f;
        if (madgwick_beta > 1.0f)
            madgwick_beta = 1.0f;

        madgwick_reset = false;
        use_stabilization = configSettings.stabilization;
        sensor_data = configSettings.sensorData;
        use_smartcorrection = configSettings.smartCorrection;
        smartcorrection_time = 0.0f;

        rot_vec = new Quaternion(0.0,0.0,0.0,1.0);
        gyro_vec = new float[3];
        accel_vec = new float[3];
        gav_vec = new float[3];
        lin_accel_vec = new float[3];
        mag_vec = new float[3];
        last_gyro_timestamp = 0;
        last_madgwick_timestamp = 0;
        last_send_timestamp = 0;
        elapsed_gyro_time = 0;
        gyro_samples = 0;
        filter_madgwick = new MadgwickAHRS(0.0f, madgwick_beta);
        filter_madgwick_quick = new MadgwickAHRS(0.0f, 0.9f);

        udpClient = udpClient_v;
        udpClient.set_listener(this);
    }

    public void register_listeners() {
        mHandler = new Handler();
        sensorManager.registerListener(this, GyroSensor, SensorManager.SENSOR_DELAY_FASTEST, mHandler);
        sensorManager.registerListener(this, AccelSensor, SensorManager.SENSOR_DELAY_FASTEST, mHandler);
        sensorManager.registerListener(this, MagSensor, SensorManager.SENSOR_DELAY_FASTEST, mHandler);
    }

    public void stop() {
        sensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.timestamp == 0)
            return;

        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE
                || event.sensor.getType() == Sensor.TYPE_GYROSCOPE_UNCALIBRATED) {
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            gyro_vec[0] += vec[0];
            gyro_vec[1] += vec[1];
            gyro_vec[2] += vec[2];
            gyro_samples++;

            if (last_gyro_timestamp != 0) {
                final float lastGyro = (event.timestamp - last_gyro_timestamp) * NS2S;
                elapsed_gyro_time += (long) (lastGyro * 1000);

                if (elapsed_gyro_time > MADGWICK_UPDATE_RATE_MS && gyro_samples > 0) {
                    gyro_vec[0] /= gyro_samples;
                    gyro_vec[1] /= gyro_samples;
                    gyro_vec[2] /= gyro_samples;

                    switch(sensor_data) {
                        case 0:
                            // Send nothing
                            break;
                        case 1:
                            // Send minimal (compat owoTrack servers)
                            udpClient.provide_owo_gyro(event.timestamp, gyro_vec);
                            break;
                        case 2:
                            if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE)  {
                                udpClient.provide_gyro(event.timestamp, gyro_vec);
                            }
                            else {
                                udpClient.provide_uncalib_gyro(event.timestamp, gyro_vec);
                            }
                            break;
                    }

                    updateMadgwick(event.timestamp);
                    elapsed_gyro_time = 0;

                    gyro_vec[0] = 0.f;
                    gyro_vec[1] = 0.f;
                    gyro_vec[2] = 0.f;
                    gyro_samples = 0;
                }
            }

            last_gyro_timestamp = event.timestamp;

        } else if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER
                || event.sensor.getType() == Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) {
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            {
                final float alpha = 0.8f;

                gav_vec[0] = lowpass_filter(alpha, gav_vec[0], vec[0]);
                gav_vec[1] = lowpass_filter(alpha, gav_vec[1], vec[1]);
                gav_vec[2] = lowpass_filter(alpha, gav_vec[2], vec[2]);

                lin_accel_vec[0] = vec[0] - gav_vec[0];
                lin_accel_vec[1] = vec[1] - gav_vec[1];
                lin_accel_vec[2] = vec[2] - gav_vec[2];
            }

            accel_vec = vec;

            switch(sensor_data) {
                case 0:
                    // Send nothing
                    break;
                case 1:
                    // Send minimal (compat owoTrack servers)
                    udpClient.provide_owo_accel(event.timestamp, lin_accel_vec);
                    break;
                case 2:
                    if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                        udpClient.provide_accel(event.timestamp, vec);
                    }
                    else {
                        udpClient.provide_uncalib_accel(event.timestamp, vec);
                    }
                    break;
            }

        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD
                || event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) {
            float[] vec = new float[3];
            vec[0] = event.values[0];
            vec[1] = event.values[1];
            vec[2] = event.values[2];

            mag_vec = vec;

            switch(sensor_data) {
                case 0:
                    // Send nothing
                    break;
                case 1:
                    // Send minimal (compat owoTrack servers)
                    break;
                case 2:
                    if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                        udpClient.provide_mag(event.timestamp, vec);
                    }
                    else {
                        udpClient.provide_uncalib_mag(event.timestamp, vec);
                    }
                    break;
            }
        }
    }

    private void updateMadgwick(long timeStamp) {
        if (last_madgwick_timestamp != 0) {
            final float deltaTime = (timeStamp - last_madgwick_timestamp) * NS2S;

            { // Main adgwick
                float beta;

                if (madgwick_reset) {
                    beta = 0.9f;
                }
                else {
                    beta = getAdaptiveBeta(deltaTime);
                }

                final float old_beta = filter_madgwick.getBeta();
                final float beta_smooth = lowpass_filter(0.1f, old_beta, beta);

                filter_madgwick.setBeta(beta_smooth);
                filter_madgwick.setSamplePeriod(deltaTime);

                if (MagSensor != null) {
                    filter_madgwick.update(
                            gyro_vec[0], gyro_vec[1], gyro_vec[2],
                            accel_vec[0], accel_vec[1], accel_vec[2],
                            mag_vec[0], mag_vec[1], mag_vec[2]);
                } else {
                    filter_madgwick.update(
                            gyro_vec[0], gyro_vec[1], gyro_vec[2],
                            accel_vec[0], accel_vec[1], accel_vec[2]);
                }
            }

            if (use_smartcorrection) { // Quick Madgwick
                filter_madgwick_quick.setSamplePeriod(deltaTime);

                if (MagSensor != null) {
                    filter_madgwick_quick.update(
                            gyro_vec[0], gyro_vec[1], gyro_vec[2],
                            accel_vec[0], accel_vec[1], accel_vec[2],
                            mag_vec[0], mag_vec[1], mag_vec[2]);
                } else {
                    filter_madgwick_quick.update(
                            gyro_vec[0], gyro_vec[1], gyro_vec[2],
                            accel_vec[0], accel_vec[1], accel_vec[2]);
                }
            }

            float[] quat = filter_madgwick.getQuaternion();
            float[] quat2 = filter_madgwick_quick.getQuaternion();
            Quaternion swapQuat = new Quaternion(
                    quat[1],
                    quat[2],
                    quat[3],
                    quat[0]
            );
            Quaternion swapQuat2 = new Quaternion(
                    quat2[1],
                    quat2[2],
                    quat2[3],
                    quat2[0]
            );

            float accel_g = 1.0f + (float)Math.sqrt(
                    lin_accel_vec[0] * lin_accel_vec[0] +
                    lin_accel_vec[1] * lin_accel_vec[1] +
                    lin_accel_vec[2] * lin_accel_vec[2]);

            // If angle difference is too big, attempt to quick reset.
            if (use_smartcorrection) {
                if (madgwick_reset) {
                    // Stop when deviation is too small
                    if (Math.abs(calculateQuaternionAngle(swapQuat, swapQuat2)) < MADGWICK_SMARTRESET_DEG_MIN) {
                        madgwick_reset = false;
                    }

                    // Stop if gravity is unstable
                    if (accel_g > MADGWICK_SMARTRESET_GRAV_STABLE && accel_g < 1.f + (1.f - MADGWICK_SMARTRESET_GRAV_STABLE)) {
                        madgwick_reset = false;
                    }
                }
                else {
                    if (Math.abs(calculateQuaternionAngle(swapQuat, swapQuat2)) > MADGWICK_SMARTRESET_DEG_MAX) {
                        // Make sure smart madgwick its kind of stable
                        if (accel_g > MADGWICK_SMARTRESET_GRAV_STABLE && accel_g < 1.f + (1.f - MADGWICK_SMARTRESET_GRAV_STABLE)) {
                            smartcorrection_time += deltaTime;

                            if (smartcorrection_time > (MADGWICK_SMARTRESET_MS / 1000.f)) {
                                if (MADGWICK_SMARTRESET_INSTANT) {
                                    swapQuat = new Quaternion(swapQuat2);
                                    smartcorrection_time = 0.0f;

                                    filter_madgwick.setQuaternion(
                                            (float)swapQuat.getW(),
                                            (float)swapQuat.getX(),
                                            (float)swapQuat.getY(),
                                            (float)swapQuat.getZ());
                                }
                                else {
                                    madgwick_reset = true;
                                }
                            }
                        }
                        else {
                            if (smartcorrection_time > 0.0f)
                                smartcorrection_time = 0.0f;
                        }
                    }
                    else {
                        if (smartcorrection_time > 0.0f)
                            smartcorrection_time = 0.0f;
                    }
                }
            }
            else {
                if (madgwick_reset)
                    madgwick_reset = false;
            }

            // For some reason ROTATION_VECTOR quaternion and MADGWICK quaternion have a 90° yaw difference.
            // Add a 90° offset to make it compatible with owoTrack servers.
            swapQuat = new Quaternion(new Vector3(0.f, 0.f, 1.f), 90.f * (Math.PI / 180.f)).mulThis(swapQuat);
            float[] newQuat = new float[4];
            newQuat[0] = (float) swapQuat.getX();
            newQuat[1] = (float) swapQuat.getY();
            newQuat[2] = (float) swapQuat.getZ();
            newQuat[3] = (float) swapQuat.getW();

            // Keep alive
            final float last_send = (timeStamp - last_send_timestamp) * NS2S;

            if (!OPTIMIZE_ROTATION_PACKET_SEND || last_send > 0.5f || !quat_equal(swapQuat, rot_vec))  {
                udpClient.provide_rot(timeStamp, newQuat);
                last_send_timestamp = timeStamp;

                rot_vec = swapQuat;
            }
        }

        last_madgwick_timestamp = timeStamp;
    }

    private boolean quat_equal(Quaternion q1, Quaternion q2)
    {
        final float EQUAL_TOLERANCE = 0.0001f;
        
        if (Math.abs(q1.getX() - q2.getX()) < EQUAL_TOLERANCE &&
                Math.abs(q1.getY() - q2.getY()) < EQUAL_TOLERANCE &&
                Math.abs(q1.getZ() - q2.getZ()) < EQUAL_TOLERANCE &&
                Math.abs(q1.getW() - q2.getW()) < EQUAL_TOLERANCE) {
            return true;
        }

        return false;
    }

    private float getAdaptiveBeta(float deltaTime) {
        if (!use_stabilization)
            return madgwick_beta;

        float gyro_sq = (float) Math.sqrt(gyro_vec[0] * gyro_vec[0] + gyro_vec[1] * gyro_vec[1] + gyro_vec[2] * gyro_vec[2]) * deltaTime;
        gyro_sq = (float) (gyro_sq * (180.f / Math.PI));

        float beta_multi = ((gyro_sq - STABILIZATION_GYRO_MIN_DEG) / STABILIZATION_GYRO_MAX_DEG);
        if (beta_multi < 0.0f)
            beta_multi = 0.f;
        if (beta_multi > 1.0f)
            beta_multi = 1.f;

        return (madgwick_beta * beta_multi);
    }


    private float lowpass_filter(float alpha, float old_val, float new_val)
    {
        return alpha * new_val + (1.f - alpha) * old_val;
    }

    private double calculateQuaternionAngle(Quaternion q1, Quaternion q2) {
        q1.normalizeThis();
        q2.normalizeThis();

        float dotProduct = (float)(q1.getX() * q2.getX() + q1.getY() * q2.getY() + q1.getZ() * q2.getZ() + q1.getW() * q2.getW());
        float angle = (float)(2 * Math.acos(Math.abs(dotProduct)));

        return Math.toDegrees(angle);
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
