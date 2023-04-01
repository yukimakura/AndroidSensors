package org.ollide.rosandroid;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;

import org.ros.concurrent.CancellableLoop;
import org.ros.internal.message.RawMessage;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import geometry_msgs.Quaternion;
import sensor_msgs.Imu;
import std_msgs.Header;

public class ImuPublisherNode extends AbstractNodeMain {
    private static float maxFrequency = 100.f;
    private float minElapse = 1000 / maxFrequency;

    //TODO Ensure that data from accelerometer, gyroscope, and orientation sensor that is published within the same message does not vary in terms of the time they are message, otherwise drop.
    private long previousPublishTime = System.currentTimeMillis();
    private boolean isAccelerometerMessagePending;
    private boolean isGyroscopeMessagePending;
    private boolean isOrientationMessagePending;

    private String topic_name;
    private SensorEventListener accelerometerListener;
    private SensorEventListener gyroscopeListener;
    private SensorEventListener orientationListener;

    private double ax, ay, az;
    private double aRoll, aPitch, aYaw;
    private double roll, pitch, yaw;
    private String imuFrameId;
    private double prevRoll, prevPitch, prevYaw;
    private OnFrameIdChangeListener imuFrameIdChangeListener;

    public ImuPublisherNode() {
        this.topic_name = "imu_data";
        isAccelerometerMessagePending = false;
        isGyroscopeMessagePending = false;
        isOrientationMessagePending = false;

        accelerometerListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent sensorEvent) {

                final double alpha = 0.8;

                double[] gravity = new double[3];
                gravity[0] = alpha * gravity[0] + (1 - alpha) * sensorEvent.values[0];
                gravity[1] = alpha * gravity[1] + (1 - alpha) * sensorEvent.values[1];
                gravity[2] = alpha * gravity[2] + (1 - alpha) * sensorEvent.values[2];

                ax = sensorEvent.values[0] - gravity[0];
                ay = sensorEvent.values[1] - gravity[1];
                az = sensorEvent.values[2] - gravity[2];
                isAccelerometerMessagePending = true;
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int i) {
            }
        };

        gyroscopeListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent sensorEvent) {
                aRoll = Math.toRadians(sensorEvent.values[2]);
                aPitch = Math.toRadians(-sensorEvent.values[0]);
                aYaw = Math.toRadians(-sensorEvent.values[1]);
                isGyroscopeMessagePending = true;
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int i) {
            }
        };

        orientationListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent sensorEvent) {

                roll = Math.toRadians(sensorEvent.values[2]);
                pitch = Math.toRadians(-sensorEvent.values[0]);
                yaw = Math.toRadians(-sensorEvent.values[1]);

                isOrientationMessagePending = true;

            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int i) {
            }
        };

        imuFrameIdChangeListener = new OnFrameIdChangeListener() {
            @Override
            public void onFrameIdChanged(String newFrameId) {
                imuFrameId = newFrameId;
            }
        };
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_android_sensors/imu_publisher_node");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        final Publisher<Imu> imuPublisher = connectedNode.newPublisher(this.topic_name, Imu._TYPE);

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            int sequenceNumber = 1;
            Header header = connectedNode.getTopicMessageFactory().newFromType(Header._TYPE);
            Imu imuMessage = imuPublisher.newMessage();

            @Override
            protected void loop() throws InterruptedException {
                long currentTimeMillis = System.currentTimeMillis();
                if (isAccelerometerMessagePending && isGyroscopeMessagePending && isOrientationMessagePending) {
                    header.setStamp(connectedNode.getCurrentTime());
                    header.setFrameId(imuFrameId);
                    header.setSeq(sequenceNumber);
                    imuMessage.setHeader(header);

                    imuMessage.getLinearAcceleration().setX(ax);
                    imuMessage.getLinearAcceleration().setY(ay);
                    imuMessage.getLinearAcceleration().setZ(az);

                    float dt = (currentTimeMillis - previousPublishTime) / 1000.f;
                    double dRoll = (roll - prevRoll);
                    if (dRoll > 180)
                        dRoll = 360 - dRoll;
                    double dPitch = (pitch - prevPitch);
                    if (dPitch > 180)
                        dPitch = 360 - dPitch;
                    double dYaw = (yaw - prevYaw);
                    if (dYaw > 180)
                        dYaw = 360 - dYaw;

                    imuMessage.getAngularVelocity().setX(dRoll / dt);
                    imuMessage.getAngularVelocity().setY(dPitch / dt);
                    imuMessage.getAngularVelocity().setZ(dYaw / dt);

                    prevRoll = roll;
                    prevPitch = pitch;
                    prevYaw = yaw;

                    quatStruct quat = fromAngles(yaw,roll,pitch);
                    imuMessage.getOrientation().setW(quat.W);
                    imuMessage.getOrientation().setX(quat.X);
                    imuMessage.getOrientation().setY(quat.Y);
                    imuMessage.getOrientation().setZ(quat.Z);

                    imuPublisher.publish(imuMessage);

                    //Wait until minimum time has elapsed
                    long elapsed = currentTimeMillis - previousPublishTime;
                    long remainingTime = (long) (minElapse - elapsed);
                    if (remainingTime > 0)
                        Thread.sleep(remainingTime);
                    previousPublishTime = System.currentTimeMillis();

                    isAccelerometerMessagePending = false;
                    isGyroscopeMessagePending = false;
                    isOrientationMessagePending = false;

                    ++this.sequenceNumber;
                } else {
                    Thread.sleep(1);
                }
            }
        });
    }

    public SensorEventListener getAccelerometerListener() {
        return accelerometerListener;
    }

    public SensorEventListener getGyroscopeListener() {
        return gyroscopeListener;
    }

    public SensorEventListener getOrientationListener() {
        return orientationListener;
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
