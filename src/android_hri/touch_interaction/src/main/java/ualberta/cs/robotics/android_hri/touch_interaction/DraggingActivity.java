package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
import android.graphics.Matrix;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.WindowManager;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

import sensor_msgs.CompressedImage;
import ualberta.cs.robotics.android_hri.touch_interaction.node.BooleanNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.Float32Node;
import ualberta.cs.robotics.android_hri.touch_interaction.node.PointNode;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.MultiTouchArea;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.gesture_detector.TwoFingerGestureDetector;


public class DraggingActivity extends RosActivity {
	
	private static final String TAG = "DraggingActivity";
    private static final String CONFIRM_TARGET="/android/confirmTarget";
    private static final String TARGET_POINT="/android/target_point";
    private static final String GRASP="/android/grasp";
    private static final String POSITION= "/android/position";
    private static final String ROTATION= "/android/rotation";
    private static final String STREAMING= "/image_converter/output_video/compressed";
    private static final String STREAMING_MSG = "sensor_msgs/CompressedImage";
    private MultiTouchArea gestureHandler = null;

    private RosImageView<CompressedImage> imageStream;
    private ImageView targetImage;
    private ImageView positionImage;
    private TextView msgText;

    private PointNode targetPointNode;
    private Float32Node graspNode;
    private PointNode positionNode;
    private Float32Node rotationNode;
    private BooleanNode confirmTargetNode;
    private String msg="";

    private boolean running = true;
    private boolean debug = true;

    public DraggingActivity() {
        super(TAG, TAG, URI.create(MainActivity.ROS_MASTER));
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {

    	Intent intent = getIntent();
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_dragging);

        targetImage = (ImageView) findViewById(R.id.targetView);
        positionImage = (ImageView) findViewById(R.id.positionView);
        msgText = (TextView) findViewById(R.id.msgTextView);

        imageStream = (RosImageView<CompressedImage>) findViewById(R.id.imageViewCenter);
        if(debug)
            imageStream.setTopicName("/usb_cam/image_raw/compressed");
        else
            imageStream.setTopicName(STREAMING);
        imageStream.setMessageType(STREAMING_MSG);
        imageStream.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        imageStream.setScaleType(ImageView.ScaleType.FIT_CENTER);

        gestureHandler = new MultiTouchArea(this, imageStream);
        gestureHandler.enableScaling();
        gestureHandler.enableDragging();
        gestureHandler.enableRotating();
        gestureHandler.enableOneFingerGestures();
        gestureHandler.enableDoubleTap();
        gestureHandler.enableLongPress();

        targetPointNode = new PointNode();
        targetPointNode.publishTo(TARGET_POINT, true, 0);
        graspNode = new Float32Node();
        graspNode.publishTo(GRASP, true, 0);
        positionNode = new PointNode();
        positionNode.publishTo(POSITION,true,0);
        rotationNode = new Float32Node();
        rotationNode.publishTo(ROTATION,true,0);
        confirmTargetNode = new BooleanNode();
        confirmTargetNode.publishTo(CONFIRM_TARGET, true, 100);

        Thread threadGestures = new Thread(){
            public void run(){
                while(running){
                    try {
                        msg="";
                        updateTarget();
                        updateConfirmTarget();
                        updatePosition();
                        updateGrasping();
                        updateRotation();
                        updateText();
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.getStackTrace();
                    }
                }
            }
        };
        threadGestures.start();

    }

    @Override
    public void onResume() {
        super.onResume();
        running=true;
    }
    
    @Override
    protected void onPause()
    {
    	super.onPause();
    }
    
    @Override
    public void onDestroy() {
        super.onDestroy();
        running=false;
    }
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        if (id == R.id.action_settings) {
            return true;
        }
        return super.onOptionsItemSelected(item);
    }

    private void updateTarget(){
        /** Update LongClick **/
        if(gestureHandler.getLongClickX()>0){

            final float x=gestureHandler.getLongClickX() - targetImage.getWidth()/2;
            final float y=gestureHandler.getLongClickY() - targetImage.getHeight()/2;

            float[] targetPoint = new float[]{gestureHandler.getLongClickX(), gestureHandler.getLongClickY()};
            float[] targetPixel = new float[2];

            Matrix streamMatrix = new Matrix();
            imageStream.getImageMatrix().invert(streamMatrix);
            streamMatrix.mapPoints(targetPixel, targetPoint);

            targetPointNode.getPublish_point()[0]=targetPixel[0];
            targetPointNode.getPublish_point()[1]=targetPixel[1];
            this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    targetImage.setX(x);
                    targetImage.setY(y);
                    targetImage.setAlpha(1.0f);
                    positionImage.setAlpha(0.f);
                    positionImage.setX(0);
                    positionImage.setY(0);
                    msg="Target Selected";
                }
            });
            if(!gestureHandler.isDetectingTwoFingerGesture()){
                gestureHandler.setDoubleDragX(0);
                positionNode.getPublish_point()[0]=0;
                positionNode.getPublish_point()[1]=0;
            }
        }
    }

    private void updatePosition(){
        if(gestureHandler.getDoubleDragX()>0){

            final float x=gestureHandler.getDoubleDragX() - positionImage.getWidth()/2;
            final float y=gestureHandler.getDoubleDragY() - positionImage.getHeight()/2;

            float[] positionPoint = new float[]{gestureHandler.getDoubleDragX(), gestureHandler.getDoubleDragY()};
            float[] positionPixel = new float[2];

            Matrix streamMatrix = new Matrix();
            imageStream.getImageMatrix().invert(streamMatrix);
            streamMatrix.mapPoints(positionPixel, positionPoint);

            positionNode.getPublish_point()[0]=positionPixel[0];
            positionNode.getPublish_point()[1]=positionPixel[1];
            this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    positionImage.setX(x);
                    positionImage.setY(y);
                    positionImage.setAlpha(0.4f);

                    targetImage.setAlpha(0.f);
                    targetImage.setX(0);
                    targetImage.setY(0);
                    msg="Position updated";
                }
            });
            gestureHandler.setLongClickX(0);
            targetPointNode.getPublish_point()[0]=0;
            targetPointNode.getPublish_point()[1]=0;
        }
    }

    private void updateConfirmTarget(){
        if(gestureHandler.getDoubleTapX()>0){
            if(gestureHandler.getLongClickX()>1){
                confirmTargetNode.setPublish_bool(true);
                confirmTargetNode.publishNow();
            }else{
                this.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(getApplicationContext(), "Select a target first! use Long Press.", Toast.LENGTH_LONG).show();
                    }
                });
            }
            gestureHandler.setDoubleTapX(0);
            gestureHandler.setDoubleTapY(0);
        }
    }

    private void updateGrasping(){
        float grasp = 3.2f*(TwoFingerGestureDetector.MAX_SCALE-gestureHandler.getScale())/(TwoFingerGestureDetector.MAX_SCALE-TwoFingerGestureDetector.MIN_SCALE);
        graspNode.setPublish_float(grasp);
    }

    private void updateRotation(){
        final float angle = gestureHandler.getAngle();
        rotationNode.setPublish_float(angle);
        if(!gestureHandler.isDetectingTwoFingerGesture()){
            gestureHandler.setAngle(0);
        }
        if(angle!=0){
            msg="Rotating: "+angle;
            gestureHandler.setDoubleDragX(0);
            targetPointNode.getPublish_point()[0]=0;
            targetPointNode.getPublish_point()[1]=0;
        }

    }

    private void updateText(){
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                msgText.setText(msg);
            }
        });
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeMainExecutor.execute(imageStream, nodeConfiguration.setNodeName(STREAMING+"sub"));

        nodeMainExecutor.execute(targetPointNode, nodeConfiguration.setNodeName(TARGET_POINT));
        nodeMainExecutor.execute(graspNode, nodeConfiguration.setNodeName(GRASP));
        nodeMainExecutor.execute(positionNode, nodeConfiguration.setNodeName(POSITION));
        nodeMainExecutor.execute(rotationNode, nodeConfiguration.setNodeName(ROTATION));
        nodeMainExecutor.execute(confirmTargetNode, nodeConfiguration.setNodeName(CONFIRM_TARGET));
    }

}
