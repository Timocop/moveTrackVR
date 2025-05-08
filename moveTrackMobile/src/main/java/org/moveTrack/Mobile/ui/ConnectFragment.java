package org.moveTrack.Mobile.ui;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Build;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.SeekBar;
import android.widget.Switch;
import android.widget.TextView;

import androidx.fragment.app.Fragment;

import org.moveTrack.Mobile.MainActivity;
import org.moveTrack.Mobile.R;
import org.moveTrack.TrackingService;

/**
 * A simple {@link Fragment} subclass.
 * Use the {@link ConnectFragment#newInstance} factory method to
 * create an instance of this fragment.
 */
public class ConnectFragment extends GenericBindingFragment {

    final static String CONN_DATA = "CONNECTION_DATA_PREF";
    Button connect_button = null;
    EditText ipAddrTxt = null;
    EditText portTxt = null;
    Switch magBox = null;
    SeekBar madgwickBetaBox = null;
    Switch stabilizationBox = null;
    RadioButton sensorDataDisabled = null;
    RadioButton sensorDataCompat = null;
    RadioButton sensorDataAll = null;
    Switch smartCorrection = null;
    SharedPreferences.OnSharedPreferenceChangeListener listener = new SharedPreferences.OnSharedPreferenceChangeListener() {
        @Override
        public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, java.lang.String s) {
            if (s.equals("ip_address")) {
                ipAddrTxt.setText(sharedPreferences.getString(s, ""));
            }
            if (s.equals("port")) {
                portTxt.setText(String.valueOf(sharedPreferences.getInt(s, 6969)));
            }
        }
    };
    public ConnectFragment() {
    }

    public static SharedPreferences get_prefs(Context c) {
        return c.getSharedPreferences(CONN_DATA, Context.MODE_PRIVATE);
    }

    public static ConnectFragment newInstance() {
        ConnectFragment fragment = new ConnectFragment();
        Bundle args = new Bundle();
        fragment.setArguments(args);
        return fragment;
    }

    public SharedPreferences get_prefs() {
        return get_prefs(getContext());
    }

    @Override
    protected void onSetStatus(String to) {
        if (curr_view == null) return;

        TextView text = curr_view.findViewById(R.id.statusText);

        if (text != null)
            text.setText(to.split("\n")[0]);
    }

    @Override
    protected void onConnectionStatus(boolean to) {
        if (connect_button != null)
            connect_button.setText(to ? "Disconnect" : "Connect");

        if (ipAddrTxt != null)
            ipAddrTxt.setEnabled(!to);

        if (portTxt != null)
            portTxt.setEnabled(!to);

        if (magBox != null)
            magBox.setEnabled(!to);

        if (madgwickBetaBox != null)
            madgwickBetaBox.setEnabled(!to);

        if (stabilizationBox != null)
            stabilizationBox.setEnabled(!to);

        if (sensorDataDisabled != null)
            sensorDataDisabled.setEnabled(!to);

        if (sensorDataAll != null)
            sensorDataAll.setEnabled(!to);

        if (sensorDataDisabled != null)
            sensorDataDisabled.setEnabled(!to);

        if (smartCorrection != null)
            smartCorrection.setEnabled(!to);
    }

    @Override
    public void onDestroy() {
        get_prefs().unregisterOnSharedPreferenceChangeListener(listener);
        save_data();

        super.onDestroy();
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        curr_view = inflater.inflate(R.layout.fragment_connect, container, false);

        connect_button = curr_view.findViewById(R.id.connectButton);
        ipAddrTxt = curr_view.findViewById(R.id.editIP);
        portTxt = curr_view.findViewById(R.id.editPort);
        magBox = curr_view.findViewById(R.id.editMagnetometer);
        madgwickBetaBox = curr_view.findViewById(R.id.seekMadgwickBeta);
        stabilizationBox = curr_view.findViewById(R.id.editStabilization);
        sensorDataDisabled = curr_view.findViewById(R.id.radioSensordata0);
        sensorDataCompat = curr_view.findViewById(R.id.radioSensordata1);
        sensorDataAll = curr_view.findViewById(R.id.radioSensordata2);
        smartCorrection = curr_view.findViewById(R.id.editSmartCorrection);


        if (!MainActivity.hasAnySensorsAtAll()) {
            connect_button.setEnabled(false);
            ipAddrTxt.setEnabled(false);
            portTxt.setEnabled(false);
            magBox.setEnabled(false);
            madgwickBetaBox.setEnabled(false);
            stabilizationBox.setEnabled(false);
            sensorDataDisabled.setEnabled(false);
            sensorDataCompat.setEnabled(false);
            sensorDataAll.setEnabled(false);
            smartCorrection.setEnabled(false);

            TextView statusText = curr_view.findViewById(R.id.statusText);
            statusText.setText(R.string.sensors_missing_all);
        } else {
            SharedPreferences prefs = get_prefs();

            ipAddrTxt.setText(prefs.getString("ip_address", ""));
            portTxt.setText(String.valueOf(prefs.getInt("port", 6969)));
            magBox.setChecked(prefs.getBoolean("magnetometer", true));
            madgwickBetaBox.setProgress((int) (prefs.getFloat("madgwickbeta", 0.1f) * 10.f));
            stabilizationBox.setChecked(prefs.getBoolean("stabilization", false));
            switch(prefs.getInt("sensordata", 0)) {
                case 0:
                    sensorDataDisabled.setChecked(true);
                    sensorDataCompat.setChecked(false);
                    sensorDataAll.setChecked(false);
                    break;
                case 1:
                    sensorDataDisabled.setChecked(false);
                    sensorDataCompat.setChecked(true);
                    sensorDataAll.setChecked(false);
                    break;
                case 2:
                    sensorDataDisabled.setChecked(false);
                    sensorDataCompat.setChecked(false);
                    sensorDataAll.setChecked(true);
                    break;
            }
            smartCorrection.setChecked(prefs.getBoolean("smartcorrection", true));

            connect_button.setOnClickListener(v -> onConnect(false));

            prefs.registerOnSharedPreferenceChangeListener(listener);

            onConnectionStatus(TrackingService.isInstanceCreated());
        }

        return curr_view;
    }

    private String get_ip_address() {
        String filtered_ip = String.valueOf(ipAddrTxt.getText()).replaceAll("[^0-9\\.]", "");
        ipAddrTxt.setText(filtered_ip);

        return filtered_ip;
    }

    private int get_port() {
        String filtered_port = String.valueOf(portTxt.getText()).replaceAll("[^0-9]", "");
        portTxt.setText(filtered_port);

        int val = 6969;
        try {
            val = Integer.parseInt(filtered_port);
        } catch (NumberFormatException ignored) {
        }

        return val;
    }

    private boolean get_mag() {
        return magBox.isChecked();
    }

    private float get_madgwickbeta() {
        return (madgwickBetaBox.getProgress() / 10.f);
    }

    private boolean get_stabilization() {
        return stabilizationBox.isChecked();
    }

    private int get_sensordata() {
        if(sensorDataDisabled.isChecked())
            return 0;
        if(sensorDataCompat.isChecked())
            return 1;
        if(sensorDataAll.isChecked())
            return 2;

        return 0;
    }
    private boolean get_smartconnection() {  return smartCorrection.isChecked(); }

    private void onConnect(boolean auto) {
        if ((service_v != null) && (service_v.is_running())) {
            onSetStatus("Killing service...");
            Intent intent = new Intent("kill-ze-service");
            getContext().sendBroadcast(intent);
            return;
        }


        onConnectionStatus(true);

        Intent mainIntent = new Intent(getContext(), TrackingService.class);
        if (auto) {
            mainIntent.putExtra("ipAddrTxt", "255.255.255.255");
        } else {
            mainIntent.putExtra("ipAddrTxt", get_ip_address());
        }

        mainIntent.putExtra("port_no", get_port());
        mainIntent.putExtra("magnetometer", get_mag());
        mainIntent.putExtra("madgwickbeta", get_madgwickbeta());
        mainIntent.putExtra("stabilization", get_stabilization());
        mainIntent.putExtra("sensordata", get_sensordata());
        mainIntent.putExtra("smartcorrection", get_smartconnection());

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            getContext().startForegroundService(mainIntent);
        } else {
            getContext().startService(mainIntent);
        }
    }

    public void save_data() {
        if (!MainActivity.hasAnySensorsAtAll())
            return;

        if (ipAddrTxt == null ||
                portTxt == null ||
                magBox == null ||
                madgwickBetaBox == null ||
                stabilizationBox == null ||
                sensorDataDisabled == null ||
                sensorDataCompat == null ||
                sensorDataAll == null ||
                smartCorrection == null)
            return;

        SharedPreferences prefs = get_prefs();
        SharedPreferences.Editor editor = prefs.edit();

        editor.putString("ip_address", get_ip_address());
        editor.putInt("port", get_port());
        editor.putBoolean("magnetometer", get_mag());
        editor.putFloat("madgwickbeta", get_madgwickbeta());
        editor.putBoolean("stabilization", get_stabilization());
        editor.putInt("sensordata", get_sensordata());
        editor.putBoolean("smartcorrection", get_smartconnection());

        editor.apply();
    }
}