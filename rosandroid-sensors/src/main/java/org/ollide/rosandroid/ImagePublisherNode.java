package org.ollide.rosandroid;

import android.util.Log;


import androidx.annotation.NonNull;

import com.otaliastudios.cameraview.frame.Frame;
import com.otaliastudios.cameraview.frame.FrameProcessor;
import com.otaliastudios.cameraview.size.Size;

import org.jboss.netty.buffer.LittleEndianHeapChannelBuffer;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

import java.io.PrintWriter;
import java.io.StringWriter;

import sensor_msgs.CompressedImage;
import std_msgs.Header;

public class ImagePublisherNode extends AbstractNodeMain {


    private ConnectedNode connectedNode;

    private Publisher<CompressedImage> imagePublisher;
    private OnFrameIdChangeListener cameraFrameIdChangeListener;
    private int sequenceNumber = 1;

    private String cameraFrameId;

    public ImagePublisherNode(){
        cameraFrameIdChangeListener = new OnFrameIdChangeListener() {
            @Override
            public void onFrameIdChanged(String newFrameId) {
                cameraFrameId = newFrameId;
            }
        };
    }

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
                    CompressedImage imageMessage = imagePublisher.newMessage();

                    header.setStamp(connectedNode.getCurrentTime());
                    header.setFrameId(cameraFrameId);
                    header.setSeq(sequenceNumber);
                    imageMessage.setHeader(header);
                    imageMessage.setFormat("jpeg");
                    imageMessage.setData(new LittleEndianHeapChannelBuffer( ImageUtil.nv21ToJpeg((byte[]) frame.getData(),receivedImageWidth,receivedImageHeight,null)));

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
        imagePublisher = connectedNode.newPublisher("image/compressed", CompressedImage._TYPE);
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

    public OnFrameIdChangeListener getFrameIdListener() {
        return cameraFrameIdChangeListener;
    }
}
