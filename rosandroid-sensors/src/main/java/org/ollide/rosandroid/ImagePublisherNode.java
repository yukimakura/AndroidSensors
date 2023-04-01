package org.ollide.rosandroid;

import android.util.Log;


import androidx.annotation.NonNull;

import com.otaliastudios.cameraview.frame.Frame;
import com.otaliastudios.cameraview.frame.FrameProcessor;
import com.otaliastudios.cameraview.size.Size;

import org.jboss.netty.buffer.BigEndianHeapChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBufferFactory;
import org.jboss.netty.buffer.ChannelBufferIndexFinder;
import org.jboss.netty.buffer.DuplicatedChannelBuffer;
import org.jboss.netty.buffer.DynamicChannelBuffer;
import org.jboss.netty.buffer.LittleEndianHeapChannelBuffer;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.GatheringByteChannel;
import java.nio.channels.ScatteringByteChannel;
import java.nio.charset.Charset;

import sensor_msgs.Imu;
import sensor_msgs.Image;
import std_msgs.Header;

public class ImagePublisherNode extends AbstractNodeMain {


    private ConnectedNode connectedNode;

    private Publisher<Image> imagePublisher;
    private int sequenceNumber = 1;
    public FrameProcessor frameProcessor = new FrameProcessor() {
        @Override
        public void process(@NonNull Frame frame) {
            Log.d("ImageNode","FrameCallback");
            long time = frame.getTime();
            Size size = frame.getSize();
            int format = frame.getFormat();

            try{
                if(connectedNode != null && size != null){

                    int receivedImageHeight = size.getHeight();
                    int receivedImageWidth = size.getWidth();

                    Header header = connectedNode.getTopicMessageFactory().newFromType(Header._TYPE);
                    Image imageMessage = imagePublisher.newMessage();

                    header.setStamp(connectedNode.getCurrentTime());
                    header.setFrameId("camera");
                    header.setSeq(sequenceNumber);
                    imageMessage.setHeader(header);
                    imageMessage.setEncoding("rgb8");
//                    imageMessage.setData(new LittleEndianHeapChannelBuffer( yuv2rgb(frame.getData(),receivedImageWidth,receivedImageHeight)));
                    imageMessage.setHeight(receivedImageHeight);
                    imageMessage.setWidth(receivedImageWidth);
                    imageMessage.setIsBigendian((byte)0);

                    Log.d("ImageNode","Publish! "+receivedImageWidth +"x"+ receivedImageHeight);
                    imagePublisher.publish(imageMessage);

                    imageMessage = null;
                }else{
                    Log.d("ImageNode", "多分Sizeがnull");
                }
            }catch (Exception ex){
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                ex.printStackTrace(pw);
                pw.flush();
                String str = sw.toString();
                Log.e("imageNode",str);
            }catch (OutOfMemoryError e) {
                Log.e("imageNode","メモリがパンパンでPublishできないよ。悲しいね。");
            }finally {
                frame = null;
                System.gc();
            }

        }
    };


    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_android_sensors/image_publisher_node");
    }


    @Override
    public void onStart(final ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        imagePublisher = connectedNode.newPublisher("image", Image._TYPE);
        connectedNode.executeCancellableLoop(new CancellableLoop() {


            @Override
            protected void loop() throws InterruptedException {

                Thread.sleep(1);
            }
        });
    }

    @Override
    public void onShutdown(final Node node) {
        this.connectedNode = connectedNode;
        Log.w("ImageNode","シャットダウン："+node.getName());
    }

    private static byte[] yuv2rgb(byte[] yuv, int width, int height) {
        int total = width * height;
        int count = 0;
        byte[] rgb = new byte[total*3];
        byte Y, Cb = 0, Cr = 0, index = 0;
        byte R, G, B;

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                Y = yuv[y * width + x];
                if (Y < 0) Y += 255;

                if ((x & 1) == 0) {
                    Cr = yuv[(y >> 1) * (width) + x + total];
                    Cb = yuv[(y >> 1) * (width) + x + total + 1];

                    if (Cb < 0) Cb += 127; else Cb -= 128;
                    if (Cr < 0) Cr += 127; else Cr -= 128;
                }

                R = (byte) (Y + Cr + (Cr >> 2) + (Cr >> 3) + (Cr >> 5));
                G = (byte) (Y - (Cb >> 2) + (Cb >> 4) + (Cb >> 5) - (Cr >> 1) + (Cr >> 3) + (Cr >> 4) + (Cr >> 5));
                B = (byte) (Y + Cb + (Cb >> 1) + (Cb >> 2) + (Cb >> 6));

                // Approximation
//				R = (int) (Y + 1.40200 * Cr);
//			    G = (int) (Y - 0.34414 * Cb - 0.71414 * Cr);
//				B = (int) (Y + 1.77200 * Cb);

                if (R < 0) R = 0; else if (R > 255) R = (byte) 255;
                if (G < 0) G = 0; else if (G > 255) G = (byte) 255;
                if (B < 0) B = 0; else if (B > 255) B = (byte) 255;

                rgb[count++] = R;
                rgb[count++] = G;
                rgb[count++] = B;
            }
        }

        return rgb;
    }
}
