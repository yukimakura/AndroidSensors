package org.ollide.rosandroid;

import android.content.Context;
import android.util.Log;

import com.benlypan.usbhid.OnUsbHidDeviceListener;
import com.benlypan.usbhid.UsbHidDevice;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import sensor_msgs.Imu;
import std_msgs.Header;

public class UsbBno055ImuPublisherNode extends AbstractNodeMain {
    private static float maxFrequency = 100.f;
    private float minElapse = 1000 / maxFrequency; //TODO Ensure that data from accelerometer, gyroscope, and orientation sensor that is published within the same message does not vary in terms of the time they are message, otherwise drop.
    private long previousPublishTime = System.currentTimeMillis();
    private String imuFrameId = "imu";
    private ConnectedNode connectedNode;
    private Publisher<Imu> imuPublisher;
    private OnFrameIdChangeListener imuFrameIdChangeListener;
    private double[] gravityBuffer = new double[3];
    private Context context;
    private int sequenceNumber = 1;
    private String latestReceiveBuffer = new String();
    private boolean isConUsbDevice = false;
    public OnBNO055Listener OnBNO055Listener = new OnBNO055Listener() {
        @Override
        public void onNewData(byte[] newData) {
            String rawReceiveString = new String(newData);
            for (String receiveString : rawReceiveString.split("\r\n")) {
                try {
                    int biasIndex = 0;
                    quatStruct pose = new quatStruct();
                    pose.X = getFloatFromBytes(newData, biasIndex);
                    biasIndex += 4;
                    pose.Y = getFloatFromBytes(newData, biasIndex);
                    biasIndex += 4;
                    pose.Z = getFloatFromBytes(newData, biasIndex);
                    biasIndex += 4;
                    pose.W = getFloatFromBytes(newData, biasIndex);
                    biasIndex += 4;

                    eularStruct linAcc = new eularStruct();
                    linAcc.X = getFloatFromBytes(newData, biasIndex);
                    biasIndex += 4;
                    linAcc.Y = getFloatFromBytes(newData, biasIndex);
                    biasIndex += 4;
                    linAcc.Z = getFloatFromBytes(newData, biasIndex);
                    biasIndex += 4;

                    eularStruct gyro = new eularStruct();
                    gyro.X = (float) Math.toRadians(getFloatFromBytes(newData, biasIndex));
                    biasIndex += 4;
                    gyro.Y = (float) Math.toRadians(getFloatFromBytes(newData, biasIndex));
                    biasIndex += 4;
                    gyro.Z = (float) Math.toRadians(getFloatFromBytes(newData, biasIndex));
                    biasIndex += 4;

                    publishProcess(pose, linAcc, gyro);

                } catch (Exception e) {
                    Log.e("BNO055Node", e.getMessage());
                }
            }
        }
    };

    public UsbBno055ImuPublisherNode(Context context) {
        this.context = context;
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
        imuPublisher = connectedNode.newPublisher("bno055/imu/data", Imu._TYPE); // Find all available drivers from attached devices.
        final int vid = 1155;
        final int pid = 22352;

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {

                UsbHidDevice device = UsbHidDevice.factory(context, vid, pid);
                if (device != null && !isConUsbDevice) {

                    device.open(context, new OnUsbHidDeviceListener() {
                        @Override
                        public void onUsbHidDeviceConnected(UsbHidDevice device) {
                            try {
                                isConUsbDevice = true;
                                while (true) {
                                    try {
                                        byte[] readBuffer = device.read(64);
                                        OnBNO055Listener.onNewData(readBuffer);
                                    } finally {
                                        Thread.sleep(1);
                                    }
                                }
                            } catch (Exception ex) {
                                Log.e("BNO055Node", ex.getMessage());
                                isConUsbDevice = false;
                            }
                        }

                        @Override
                        public void onUsbHidDeviceConnectFailed(UsbHidDevice device) {
                            Log.e("BNO055Node","DisCon");
                            isConUsbDevice = false;
                        }
                    });
                    //この遅延をいれないと、次のループでまたオープンしてしまうため
                    Thread.sleep(500);

                }

                Thread.sleep(1);
            }
        });
    }

    public OnFrameIdChangeListener getFrameIdListener() {
        return imuFrameIdChangeListener;
    }

    private void publishProcess(quatStruct pose, eularStruct linearAcc, eularStruct gyro) {
        Header header = connectedNode.getTopicMessageFactory().newFromType(Header._TYPE);
        Imu imuMessage = imuPublisher.newMessage();
        header.setStamp(connectedNode.getCurrentTime());
        header.setFrameId(imuFrameId);
        header.setSeq(sequenceNumber++);
        imuMessage.setHeader(header);

        imuMessage.getLinearAcceleration().setX(linearAcc.X);
        imuMessage.getLinearAcceleration().setY(linearAcc.Y);
        imuMessage.getLinearAcceleration().setZ(linearAcc.Z);
        imuMessage.getAngularVelocity().setX(gyro.X);
        imuMessage.getAngularVelocity().setY(gyro.Y);
        imuMessage.getAngularVelocity().setZ(gyro.Z);
        imuMessage.getOrientation().setX(pose.X);
        imuMessage.getOrientation().setY(pose.Y);
        imuMessage.getOrientation().setZ(pose.Z);
        imuMessage.getOrientation().setW(pose.W);
        imuPublisher.publish(imuMessage);

    }

    private quatStruct fromAngles(float yaw, float roll, float pitch) {
        float angle;
        float sinRoll, sinPitch, sinYaw, cosRoll, cosPitch, cosYaw;
        angle = pitch * 0.5f;
        sinPitch = (float) Math.sin(angle);
        cosPitch = (float) Math.cos(angle);
        angle = roll * 0.5f;
        sinRoll = (float) Math.sin(angle);
        cosRoll = (float) Math.cos(angle);
        angle = yaw * 0.5f;
        sinYaw = (float) Math.sin(angle);
        cosYaw = (float) Math.cos(angle); // variables used to reduce multiplication calls.
        float cosRollXcosPitch = cosRoll * cosPitch;
        float sinRollXsinPitch = sinRoll * sinPitch;
        float cosRollXsinPitch = cosRoll * sinPitch;
        float sinRollXcosPitch = sinRoll * cosPitch;
        quatStruct quat = new quatStruct();
        quat.W = (cosRollXcosPitch * cosYaw - sinRollXsinPitch * sinYaw);
        quat.X = (cosRollXcosPitch * sinYaw + sinRollXsinPitch * cosYaw);
        quat.Y = (sinRollXcosPitch * cosYaw + cosRollXsinPitch * sinYaw);
        quat.Z = (cosRollXsinPitch * cosYaw - sinRollXcosPitch * sinYaw);
        float n = (float) Math.sqrt(1f / norm(quat));
        quat.W *= n;
        quat.X *= n;
        quat.Y *= n;
        quat.Z *= n;
        return quat;
    }

    public double norm(quatStruct quat) {
        return quat.W * quat.W + quat.X * quat.X + quat.Y * quat.Y + quat.Z * quat.Z;
    }

    private class quatStruct {
        public float W;
        public float X;
        public float Y;
        public float Z;
    }

    private class eularStruct {
        public float X;
        public float Y;
        public float Z;

    }

    private float getFloatFromBytes(byte[] rawBytes, int biasIndex) {
        int intBit = (int)Long.parseLong(String.format("%02X%02X%02X%02X", rawBytes[biasIndex + 0], rawBytes[biasIndex + 1], rawBytes[biasIndex + 2], rawBytes[biasIndex + 3]), 16);
        return Float.intBitsToFloat(intBit);
    }
}