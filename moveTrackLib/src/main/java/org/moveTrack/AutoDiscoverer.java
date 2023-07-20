package org.moveTrack;

import android.app.Activity;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Build;
import android.view.WindowManager;

import androidx.appcompat.app.AlertDialog;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;


public class AutoDiscoverer {
    static final int port = 35903;
    //static final String mcastAddress = "234.35.90.3";
    // VPNs on Android will cause some issues with multicast, need to just broadcast :(
    static final String mcastAddress = "255.255.255.255";
    public static boolean discoveryStillNecessary = true;
    public static boolean dialogShown = false;
    Activity act;
    ConfigSaver onConnect;
    public AutoDiscoverer(Activity c, ConfigSaver onYes) {
        act = c;
        onConnect = onYes;
    }

    private static DiscoveryResult attempt_discover_infoserver(int timeout) {
        try {
            DatagramSocket socket = new DatagramSocket();
            {
                String handshake = "DISCOVERY";
                DatagramPacket packet = new DatagramPacket(handshake.getBytes(), handshake.length(),
                        InetAddress.getByName(mcastAddress), port);

                socket.send(packet);

                socket.setBroadcast(true);

                ByteBuffer buff = ByteBuffer.allocate(128);
                DatagramPacket pkt = new DatagramPacket(buff.array(), buff.capacity());
                socket.setSoTimeout(timeout);
                socket.receive(pkt);


                String response = new String(buff.array());


                for (String line : response.split("\n")) {
                    try {
                        int port = Integer.parseInt(line.split(":")[0]);
                        String name = line.split(":")[1];

                        if (port > 0) {
                            //discoveryStillNecessary = false;
                            //alert(pkt.getAddress(), port, name);
                            return DiscoveryResult.some(pkt.getAddress(), port, name);
                        }
                    } catch (NumberFormatException ignored) {
                    }
                }
            }
        } catch (SocketTimeoutException ignored) {
        } catch (Exception e) {
            e.printStackTrace();
        }

        return DiscoveryResult.none();
    }

    public static DiscoveryResult attempt_discover(int timeout) {
        DiscoveryResult info_result = attempt_discover_infoserver(timeout);
        if (info_result.found) return info_result;

        return DiscoveryResult.none();
    }

    private void plsDoConnect(InetAddress addr, int port) {
        Intent mainIntent = new Intent(act, TrackingService.class);
        mainIntent.putExtra("ipAddrTxt", addr.getHostAddress());
        mainIntent.putExtra("port_no", port);

        ConfigSettings configSettings = onConnect.saveLoad(addr.getHostAddress(), port);

        mainIntent.putExtra("magnetometer", configSettings.magnetometerEnabled);
        mainIntent.putExtra("madgwickbeta", configSettings.madgwickBeta);
        mainIntent.putExtra("stabilization", configSettings.stabilization);
        mainIntent.putExtra("rawsensor", configSettings.rawSensors);

        // start service
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            act.startForegroundService(mainIntent);
        } else {
            act.startService(mainIntent);
        }
    }

    private void alert(InetAddress addr, int port, String name) {
        dialogShown = true;
        act.runOnUiThread(() -> {
            try {
                AlertDialog.Builder alert = new AlertDialog.Builder(act);
                alert.setTitle("Automatic Discovery");
                alert.setMessage("Connect to " + addr.getHostAddress() + ":" + String.valueOf(port) + " (" + name + ")?");

                alert.setPositiveButton("Yes", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int whichButton) {
                        plsDoConnect(addr, port);
                    }
                });

                alert.setNegativeButton("No", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int whichButton) {
                    }
                });

                alert.show();
            } catch (WindowManager.BadTokenException ignored) {
            }
        });
    }

    public void try_discover() {
        while (discoveryStillNecessary) {
            DiscoveryResult result = attempt_discover(5000);

            if (result.found) {
                discoveryStillNecessary = false;
                alert(result.server_address, result.port, result.name);
                break;
            }
        }
    }


    public interface ConfigSaver {
        ConfigSettings saveLoad(String ip_addr, int port);
    }

    public static class ConfigSettings {
        public boolean magnetometerEnabled;
        public float madgwickBeta;
        public boolean stabilization;
        public boolean rawSensors;
    }

    public static class DiscoveryResult {
        public boolean found;

        public InetAddress server_address;
        public int port;

        public String name;

        DiscoveryResult() {
        }

        public static DiscoveryResult none() {
            DiscoveryResult result = new DiscoveryResult();
            result.found = false;

            return result;
        }

        public static DiscoveryResult some(InetAddress srv, int port, String name) {
            DiscoveryResult result = new DiscoveryResult();
            result.found = true;
            result.server_address = srv;
            result.port = port;
            result.name = name;

            return result;
        }
    }

}
