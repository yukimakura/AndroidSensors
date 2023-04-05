package org.ollide.rosandroid;

import static android.content.Context.USB_SERVICE;
import static androidx.core.content.ContextCompat.getSystemService;


import android.app.AlertDialog;
import android.content.Context;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Bundle;

import android.content.Context;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import org.json.JSONException;
import org.json.JSONObject;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.List;

import sensor_msgs.CompressedImage;
import sensor_msgs.Imu;
import std_msgs.Header;

public class UsbBno055ImuPublisherNode extends AbstractNodeMain {
    private static float maxFrequency = 100.f;
    private float minElapse = 1000 / maxFrequency;

    //TODO Ensure that data from accelerometer, gyroscope, and orientation sensor that is published within the same message does not vary in terms of the time they are message, otherwise drop.
    private long previousPublishTime = System.currentTimeMillis();
    private String imuFrameId;

    private ConnectedNode connectedNode;

    private Publisher<Imu> imuPublisher;
    private OnFrameIdChangeListener imuFrameIdChangeListener;
    private double[] gravityBuffer = new double[3];

    private UsbManager manager;

    public OnBNO055Listener OnBNO055Listener = new OnBNO055Listener() {
        @Override
        public void onNewData(byte[] newData) {
            try {
                Log.d("BNO055Node",newData.toString());
                JSONObject jsonObj = new JSONObject(newData.toString());
            } catch (JSONException e) {
                throw new RuntimeException(e);
            }
        }
    };

    public UsbBno055ImuPublisherNode(UsbManager manager) {
        this.manager = manager;

        imuFrameIdChangeListener = new OnFrameIdChangeListener() {
            @Override
            public void onFrameIdChanged(String newFrameId) {
                imuFrameId = newFrameId;
            }
        };
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_android_sensors/bno055_imu_publisher_node");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        imuPublisher = connectedNode.newPublisher("bno055/imu/data", Imu._TYPE);

        // Find all available drivers from attached devices.
        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager);
        if (availableDrivers.isEmpty()) {
            return;
        }

        // Open a connection to the first available driver.
        UsbSerialDriver driver = availableDrivers.get(0);
        UsbDeviceConnection connection = manager.openDevice(driver.getDevice());
        if (connection == null) {
            // add UsbManager.requestPermission(driver.getDevice(), ..) handling here
            return;
        }


        final UsbSerialPort port = driver.getPorts().get(0); // Most devices have just one port (port 0)
        try {
            port.open(connection);
            port.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
            if (port != null) {
                // シリアル通信マネージャと、シリアルポート、イベント受信時のコールバックを紐づける
                final SerialInputOutputManager serIoManager = new SerialInputOutputManager(port, new SerialInputOutputManager.Listener() {
                    @Override
                    public void onNewData(byte[] bytes) {
                        OnBNO055Listener.onNewData(bytes);
                    }

                    @Override
                    public void onRunError(Exception e) {
                        Log.e("BNO055Node",e.getMessage());
                    }
                });
                serIoManager.start();
                Log.i("BNO055Node",serIoManager.getState().toString());



            } else {
                // 適当にエラーハンドリング
                Log.e("BNO055Node","オープンに失敗");
            }
        }catch (Exception ex){
            Log.e("BNO055Node","デバイスのオープンに失敗しました");
        }

        connectedNode.executeCancellableLoop(new CancellableLoop() {


            @Override
            protected void loop() throws InterruptedException {

                Thread.sleep(1);
            }
        });
    }


    public OnFrameIdChangeListener getFrameIdListener() {
        return imuFrameIdChangeListener;
    }

    private quatStruct fromAngles(double yaw, double roll, double pitch) {
        double angle;
        double sinRoll, sinPitch, sinYaw, cosRoll, cosPitch, cosYaw;
        angle = pitch * 0.5f;
        sinPitch = Math.sin(angle);
        cosPitch = Math.cos(angle);
        angle = roll * 0.5f;
        sinRoll = Math.sin(angle);
        cosRoll = Math.cos(angle);
        angle = yaw * 0.5f;
        sinYaw = Math.sin(angle);
        cosYaw = Math.cos(angle);

        // variables used to reduce multiplication calls.
        double cosRollXcosPitch = cosRoll * cosPitch;
        double sinRollXsinPitch = sinRoll * sinPitch;
        double cosRollXsinPitch = cosRoll * sinPitch;
        double sinRollXcosPitch = sinRoll * cosPitch;

        quatStruct quat = new quatStruct();
        quat.W = (cosRollXcosPitch * cosYaw - sinRollXsinPitch * sinYaw);
        quat.X = (cosRollXcosPitch * sinYaw + sinRollXsinPitch * cosYaw);
        quat.Y = (sinRollXcosPitch * cosYaw + cosRollXsinPitch * sinYaw);
        quat.Z = (cosRollXsinPitch * cosYaw - sinRollXcosPitch * sinYaw);


        double n = Math.sqrt(1f / norm(quat));
        quat.W *= n;
        quat.X *= n;
        quat.Y *= n;
        quat.Z *= n;

        return quat;
    }

    public double norm(quatStruct quat) {
        return quat.W * quat.W + quat.X * quat.X + quat.Y * quat.Y + quat.Z * quat.Z;
    }

    private class quatStruct{
        public double W;
        public double X;
        public double Y;
        public double Z;
    }
}
