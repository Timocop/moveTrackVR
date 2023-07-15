package org.moveTrack.Mobile;


import androidx.appcompat.app.AppCompatActivity;
import androidx.navigation.NavController;
import androidx.navigation.Navigation;
import androidx.navigation.ui.NavigationUI;

import android.content.Context;
import android.content.SharedPreferences;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Bundle;

import com.google.android.material.bottomnavigation.BottomNavigationView;

import org.moveTrack.*;
import org.moveTrack.Mobile.ui.ConnectFragment;

public class MainActivity extends AppCompatActivity {

    public static boolean[] sensor_exist;
    public static boolean getSensorExists(int sensor){
        if((sensor < 0) || (sensor >= 3)) return false;
        return sensor_exist[sensor];
    }

    public static boolean hasAnySensorsAtAll(){
        return getSensorExists(0) || getSensorExists(1);
    }



    private static String missingSensorMessage = "";
    public static String getSensorText(){
        return missingSensorMessage;
    }

    private void fillSensorArray(){
        SensorManager man = (SensorManager)getSystemService(Context.SENSOR_SERVICE);

        sensor_exist = new boolean[3];

        sensor_exist[0] = (
                man.getDefaultSensor(Sensor.TYPE_GYROSCOPE) != null ||
                man.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) != null
        );
        sensor_exist[1] = (
                man.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) != null ||
                man.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) != null
        );
        sensor_exist[2] = man.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) != null;

        missingSensorMessage = "";

        if(!hasAnySensorsAtAll()){
            missingSensorMessage = getString(R.string.sensors_missing_all);
        }

    }

    private void ensureUUIDSet(){
        SharedPreferences prefs = getSharedPreferences("FakeMAC", Context.MODE_PRIVATE);

        long val = -1;
        if(!prefs.contains("FakeMACValue")){
            SharedPreferences.Editor editor = prefs.edit();
            val = (new java.util.Random()).nextLong();
            editor.putLong("FakeMACValue", val);
            editor.apply();
        }else{
            val = prefs.getLong("FakeMACValue", 1);
        }

        Handshaker.setMac(val);
    }

    public static NavController contr;

    private void connect(String ip, int port){
        SharedPreferences prefs = ConnectFragment.get_prefs(this);
        SharedPreferences.Editor editor = prefs.edit();

        editor.putString("ip_address", ip);
        editor.putInt("port", port);

        editor.apply();


        contr.navigate(R.id.connectFragment);
    }

    private void runDiscovery(){
        if(!hasAnySensorsAtAll()) return;
        if(!AutoDiscoverer.discoveryStillNecessary) return;

        try {
            AutoDiscoverer disc = new AutoDiscoverer(this, this::connect);
            Thread thrd = new Thread(disc::try_discover);
            thrd.start();
        } catch(OutOfMemoryError ignored){}
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        ensureUUIDSet();
        fillSensorArray();

        setContentView(R.layout.activity_main);

        contr = Navigation.findNavController(this, R.id.fragment);

        BottomNavigationView nav = findViewById(R.id.nav_view);

        NavigationUI.setupWithNavController(nav, contr);

        runDiscovery();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    protected void onResume() {
        super.onResume();
    }
}