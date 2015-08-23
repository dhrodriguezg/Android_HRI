package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.Matrix;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.WindowManager;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

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


public class DraggingActivity extends RosActivity implements SensorEventListener {
	
	private static final String TAG = "DraggingActivity";
    private static final String STREAMING= "/image_converter/output_video/compressed";
    private static final String STREAMING_MSG = "sensor_msgs/CompressedImage";
    private static final String EMERGENCY_STOP = "/android/emergency_stop";
    private static final String ENABLE_VS = "/android/enable_vs";
    private static final String TARGET_POINT="/android/target_point";
    private static final String ROTATION= "/android/rotation";
    private static final String GRASP="/android/grasping_abs";

    private NodeMainExecutor nodeMain;

    private static final float ROLL_THREASHOLD = 6.0f;
    private static final float PITCH_THREASHOLD = 8.5f;
    private static final float MAX_GRASP = 2.0f;

    private MultiTouchArea gestureHandler = null;

    private RosImageView<CompressedImage> imageStream;
    private ImageView targetImage;
    private ImageView positionImage;
    private TextView msgText;
    private TextView pitchText;
    private TextView rollText;

    private PointNode targetPointNode;
    private Float32Node graspNode;
    private PointNode rotationNode;
    private BooleanNode emergencyNode;
    private BooleanNode vsNode;

    private String msg="";
    private boolean running = true;
    private boolean debug = true;

    private SensorManager senSensorManager;
    private Sensor senAccelerometer;

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
        pitchText = (TextView) findViewById(R.id.textRotPitch);
        rollText = (TextView) findViewById(R.id.textRotRoll);

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
        targetPointNode.publishTo(TARGET_POINT, false, 10);

        graspNode = new Float32Node();
        graspNode.publishTo(GRASP, false, 2);
        graspNode.setPublishFreq(500);

        rotationNode = new PointNode();
        rotationNode.publishTo(ROTATION, false, 10);

        emergencyNode = new BooleanNode();
        emergencyNode.publishTo(EMERGENCY_STOP, true, 0);
        emergencyNode.setPublishFreq(100);
        emergencyNode.setPublish_bool(true);
        //emergencyNode.publishNow();

        vsNode = new BooleanNode();
        vsNode.publishTo(ENABLE_VS, true, 0);
        vsNode.setPublish_bool(false);

        senSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        senAccelerometer = senSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        senSensorManager.registerListener(this, senAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);


        ToggleButton emergencyStop = (ToggleButton)findViewById(R.id.emergencyButton) ;
        emergencyStop.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                if (isChecked) {
                    Toast.makeText(getApplicationContext(), "EMERGENCY STOP ACTIVATED!", Toast.LENGTH_LONG).show();
                    imageStream.setBackgroundColor(Color.RED);
                    emergencyNode.setPublish_bool(false);
                } else {
                    Toast.makeText(getApplicationContext(), "EMERGENCY STOP DEACTIVATED!", Toast.LENGTH_LONG).show();
                    imageStream.setBackgroundColor(Color.GREEN);
                    emergencyNode.setPublish_bool(true);
                }
            }
        });

        updateGrasping();
        msg="";
        Thread threadGestures = new Thread(){
            public void run(){
                while(running){
                    try {
                        TwoFingerGestureDetector.MAX_RESOLUTION=imageStream.getWidth();
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
        emergencyNode.setPublish_bool(true);
        running=true;
    }
    
    @Override
    protected void onPause()
    {
        emergencyNode.setPublish_bool(false);
    	super.onPause();
    }
    
    @Override
    public void onDestroy() {
        emergencyNode.setPublish_bool(false);
        vsNode.setPublish_bool(false);
        nodeMain.shutdown();
        running=false;
        super.onDestroy();
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

            if(!validTarget(targetPixel[0],targetPixel[1])){
                gestureHandler.setLongClickX(0);
                return;
            }
            targetPointNode.getPublish_point()[0]=targetPixel[0];
            targetPointNode.getPublish_point()[1]=targetPixel[1];
            vsNode.setPublish_bool(false);

            this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    targetImage.setX(x);
                    targetImage.setY(y);
                    targetImage.setAlpha(1.0f);
                    positionImage.setAlpha(0.f);
                    positionImage.setX(0);
                    positionImage.setY(0);
                    msg = String.format("Target (%d , %d) ",(int)targetPointNode.getPublish_point()[0],(int)targetPointNode.getPublish_point()[1]);
                }
            });
            gestureHandler.setLongClickX(0);
            if(!gestureHandler.isDetectingTwoFingerGesture()){
                gestureHandler.setDoubleDragX(0);
            }
        }
    }

    private void updateConfirmTarget(){
        if(gestureHandler.getDoubleTapX()>0){
            if(targetPointNode.getPublish_point()[0] > 1){
                targetPointNode.publishNow();
                vsNode.setPublish_bool(true);
                msg = String.format("Going to target (%d , %d) ",(int)targetPointNode.getPublish_point()[0],(int)targetPointNode.getPublish_point()[1]);
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

    private void updatePosition(){
        if(gestureHandler.getDoubleDragX()>0){

            final float x=gestureHandler.getDoubleDragX() - positionImage.getWidth()/2;
            final float y=gestureHandler.getDoubleDragY() - positionImage.getHeight()/2;

            float[] positionPoint = new float[]{gestureHandler.getDoubleDragX(), gestureHandler.getDoubleDragY()};
            float[] positionPixel = new float[2];

            Matrix streamMatrix = new Matrix();
            imageStream.getImageMatrix().invert(streamMatrix);
            streamMatrix.mapPoints(positionPixel, positionPoint);

            if(!validTarget(positionPixel[0],positionPixel[1])){
                gestureHandler.setDoubleDragX(0);
                return;
            }

            targetPointNode.getPublish_point()[0]=positionPixel[0];
            targetPointNode.getPublish_point()[1]=positionPixel[1];
            targetPointNode.publishNow();
            vsNode.setPublish_bool(true);
            this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    positionImage.setX(x);
                    positionImage.setY(y);
                    positionImage.setAlpha(0.4f);

                    targetImage.setAlpha(0.f);
                    targetImage.setX(0);
                    targetImage.setY(0);
                    msg = String.format("Moving to (%d , %d) ", (int) targetPointNode.getPublish_point()[0], (int) targetPointNode.getPublish_point()[1]);
                }
            });
            gestureHandler.setDoubleDragX(0);
            gestureHandler.setLongClickX(0);
        }
    }

    private void updateRotation(){
        final float angle = gestureHandler.getAngle()*3.1416f/180f;
        if(!gestureHandler.isDetectingTwoFingerGesture()){
            gestureHandler.setAngle(0);
        }
        if(angle!=0){

            if (rollText.getAlpha()>0.9f){
                msg = String.format("Rotating Roll += %.2f ",angle);
                rotationNode.getPublish_point()[0]=angle;
                rotationNode.getPublish_point()[1]=0;

            }else{
                msg = String.format("Rotating Pitch += %.2f ",angle);
                rotationNode.getPublish_point()[0]=0;
                rotationNode.getPublish_point()[1]=angle;
            }
            gestureHandler.setDoubleDragX(0);
            rotationNode.publishNow();
            vsNode.setPublish_bool(false);
            this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    positionImage.setAlpha(0.0f);
                    targetImage.setAlpha(0.0f);
                }
            });
        }

    }

    private void updateGrasping(){
        float grasp = MAX_GRASP*(TwoFingerGestureDetector.MAX_SCALE-gestureHandler.getScale())/(TwoFingerGestureDetector.MAX_SCALE-TwoFingerGestureDetector.MIN_SCALE);
        if(grasp!=graspNode.getPublish_float()){
            graspNode.setPublish_float(grasp);
            msg = String.format("Grasping = %.2f ",grasp);
            this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    positionImage.setAlpha(0.0f);
                    targetImage.setAlpha(0.0f);
                }
            });
        }
        graspNode.publishNow();
    }

    private void updateText(){
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                msgText.setText(msg);
            }
        });
    }

    private boolean validTarget(float x, float y){
        if (x < 0 || y < 0)
            return false;
        if (x > imageStream.getDrawable().getIntrinsicWidth() ||  y > imageStream.getDrawable().getIntrinsicHeight())
            return false;
        return true;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {


        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            float x = event.values[0];
            float y = event.values[1];
            float z = event.values[2];

            z=y-1; //for testing with shield TODO
            y=x-1;

            if(Math.abs(y)>ROLL_THREASHOLD){
                rollText.setAlpha(1.f);
                pitchText.setAlpha(.1f);
            }
            if(Math.abs(z)>PITCH_THREASHOLD) {
                rollText.setAlpha(.1f);
                pitchText.setAlpha(1.f);
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        nodeMain=nodeMainExecutor;
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeMainExecutor.execute(imageStream, nodeConfiguration.setNodeName(STREAMING+"sub"));

        nodeMainExecutor.execute(targetPointNode, nodeConfiguration.setNodeName(TARGET_POINT));
        nodeMainExecutor.execute(graspNode, nodeConfiguration.setNodeName(GRASP));
        nodeMainExecutor.execute(rotationNode, nodeConfiguration.setNodeName(ROTATION));
        nodeMainExecutor.execute(emergencyNode, nodeConfiguration.setNodeName(EMERGENCY_STOP));
        nodeMainExecutor.execute(vsNode, nodeConfiguration.setNodeName(ENABLE_VS));
    }

}