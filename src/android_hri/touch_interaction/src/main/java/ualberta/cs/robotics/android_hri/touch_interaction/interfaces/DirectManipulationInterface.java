package ualberta.cs.robotics.android_hri.touch_interaction.interfaces;

import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.Matrix;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
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
import org.ros.android.NodeMainExecutorService;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

import sensor_msgs.CompressedImage;
import ualberta.cs.robotics.android_hri.touch_interaction.MainActivity;
import ualberta.cs.robotics.android_hri.touch_interaction.R;
import ualberta.cs.robotics.android_hri.touch_interaction.topic.BooleanTopic;
import ualberta.cs.robotics.android_hri.touch_interaction.topic.Float32Topic;
import ualberta.cs.robotics.android_hri.touch_interaction.topic.Int32Topic;
import ualberta.cs.robotics.android_hri.touch_interaction.topic.PointTopic;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.MultiTouchArea;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.gesture_detector.TwoFingerGestureDetector;
import ualberta.cs.robotics.android_hri.touch_interaction.utils.AndroidNode;


public class DirectManipulationInterface extends RosActivity implements SensorEventListener {
	
	private static final String TAG = "DirectManipulationInterface";
    private static final String NODE_NAME="/android/"+TAG.toLowerCase();

    private static final String STREAMING= "/image_converter/output_video/compressed";
    private static final String STREAMING_MSG = "sensor_msgs/CompressedImage";
    private static final String EMERGENCY_STOP = "/android/emergency_stop";
    private static final String INTERFACE_NUMBER="/android/interface_number";
    private static final String TARGET_POINT="/android/target_point";
    private static final String POSITION="/android/position_abs";

    private static final String ENABLE_VS = "/android/enable_vs";
    private static final String ROTATION= "/android/rotation_rel";
    private static final String GRASP="/android/grasping_abs";

    private NodeMainExecutorService nodeMain;

    private static final float MAX_GRASP = 2.0f;

    private MultiTouchArea gestureHandler = null;

    private RosImageView<CompressedImage> imageStream;
    private ImageView targetImage;
    private ImageView positionImage;
    private TextView msgText;

    private TextView moveStatus;
    private TextView rotateStatus;
    private TextView graspStatus;

    private AndroidNode androidNode;
    private PointTopic targetPointNode;
    private PointTopic positionNode;
    private Float32Topic graspNode;
    private Int32Topic interfaceNumberNode;
    private PointTopic rotationNode;
    private BooleanTopic emergencyNode;
    private BooleanTopic vsNode;

    private String msg="";
    private boolean running = true;
    private boolean debug = true;

    private SensorManager senSensorManager;
    private Sensor senAccelerometer;

    public DirectManipulationInterface() {
        super(TAG, TAG, URI.create(MainActivity.ROS_MASTER));
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {

    	Intent intent = getIntent();
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.interface_directmanipulation);

        targetImage = (ImageView) findViewById(R.id.targetView);
        positionImage = (ImageView) findViewById(R.id.positionView);
        msgText = (TextView) findViewById(R.id.msgTextView);
        moveStatus = (TextView) findViewById(R.id.moveStatus);
        rotateStatus = (TextView) findViewById(R.id.rotateStatus);
        graspStatus = (TextView) findViewById(R.id.graspStatus);

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
        //gestureHandler.enableDragging();
        gestureHandler.enableScroll(); //single dragging
        gestureHandler.enableRotating();
        gestureHandler.enableOneFingerGestures();
        //Disabled Select target gestures
        //gestureHandler.enableDoubleTap();
        //gestureHandler.enableLongPress();

        targetPointNode = new PointTopic();
        targetPointNode.publishTo(TARGET_POINT, false, 10);

        positionNode = new PointTopic();
        positionNode.publishTo(POSITION, false, 10);

        graspNode = new Float32Topic();
        graspNode.publishTo(GRASP, false, 2);
        graspNode.setPublishingFreq(500);

        rotationNode = new PointTopic();
        rotationNode.publishTo(ROTATION, false, 10);

        interfaceNumberNode = new Int32Topic();
        interfaceNumberNode.publishTo(INTERFACE_NUMBER, true, 0);
        interfaceNumberNode.setPublishingFreq(100);
        interfaceNumberNode.setPublisher_int(3);

        emergencyNode = new BooleanTopic();
        emergencyNode.publishTo(EMERGENCY_STOP, true, 0);
        emergencyNode.setPublishingFreq(100);
        emergencyNode.setPublisher_bool(true);

        vsNode = new BooleanTopic();
        vsNode.publishTo(ENABLE_VS, true, 0);
        vsNode.setPublisher_bool(false);

        androidNode = new AndroidNode(NODE_NAME);
        androidNode.addTopics(targetPointNode,positionNode,graspNode,rotationNode,emergencyNode,vsNode,interfaceNumberNode);
        androidNode.addNodeMain(imageStream);

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
                    emergencyNode.setPublisher_bool(false);
                } else {
                    Toast.makeText(getApplicationContext(), "EMERGENCY STOP DEACTIVATED!", Toast.LENGTH_LONG).show();
                    imageStream.setBackgroundColor(Color.TRANSPARENT);
                    emergencyNode.setPublisher_bool(true);
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
        emergencyNode.setPublisher_bool(true);
        running=true;
    }
    
    @Override
    protected void onPause() {
        emergencyNode.setPublisher_bool(false);
    	super.onPause();
    }
    
    @Override
    public void onDestroy() {
        emergencyNode.setPublisher_bool(false);
        vsNode.setPublisher_bool(false);
        nodeMain.forceShutdown();
        running=false;
        super.onDestroy();
    }
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        return super.onOptionsItemSelected(item);
    }

    private void updateTarget() {

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
            targetPointNode.getPublisher_point()[0]=targetPixel[0];
            targetPointNode.getPublisher_point()[1]=targetPixel[1];
            vsNode.setPublisher_bool(false);

            this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    targetImage.setX(x);
                    targetImage.setY(y);
                    targetImage.setAlpha(1.0f);
                    positionImage.setAlpha(0.f);
                    positionImage.setX(0);
                    positionImage.setY(0);
                    msg = String.format("Target (%d , %d) ",(int)targetPointNode.getPublisher_point()[0],(int)targetPointNode.getPublisher_point()[1]);
                }
            });
            gestureHandler.setLongClickX(0);
            if(!gestureHandler.isDetectingTwoFingerGesture()){
                gestureHandler.setDoubleDragX(0);
            }
        }
    }

    private void updateConfirmTarget() {
        if(gestureHandler.getDoubleTapX()>0) {
            if(targetPointNode.getPublisher_point()[0] > 1) {
                targetPointNode.publishNow();
                vsNode.setPublisher_bool(true);
                msg = String.format("Going to target (%d , %d) ",(int)targetPointNode.getPublisher_point()[0],(int)targetPointNode.getPublisher_point()[1]);
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

    private void updatePosition() {
        if(gestureHandler.getSingleDragX()>0){

            final float x=gestureHandler.getSingleDragX() - positionImage.getWidth()/2;
            final float y=gestureHandler.getSingleDragY() - positionImage.getHeight()/2;

            float[] positionPoint = new float[]{gestureHandler.getSingleDragX(), gestureHandler.getSingleDragY()};
            float[] positionPixel = new float[2];

            Matrix streamMatrix = new Matrix();
            imageStream.getImageMatrix().invert(streamMatrix);
            streamMatrix.mapPoints(positionPixel, positionPoint);

            if(!validTarget(positionPixel[0],positionPixel[1])){
                gestureHandler.setSingleDragX(0);
                return;
            }

            positionNode.getPublisher_point()[0] = MainActivity.WORKSPACE_Y_OFFSET - positionPixel[1]*MainActivity.WORKSPACE_HEIGHT/(float)imageStream.getDrawable().getIntrinsicHeight();
            positionNode.getPublisher_point()[1] = MainActivity.WORKSPACE_X_OFFSET - positionPixel[0]*MainActivity.WORKSPACE_WIDTH/(float)imageStream.getDrawable().getIntrinsicWidth();
            positionNode.getPublisher_point()[2] = 0;
            positionNode.publishNow();

            this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    positionImage.setX(x);
                    positionImage.setY(y);
                    positionImage.setAlpha(0.4f);

                    targetImage.setAlpha(0.f);
                    targetImage.setX(0);
                    targetImage.setY(0);
                    msg = String.format("Moving to (%.4f , %.4f) ", positionNode.getPublisher_point()[0], positionNode.getPublisher_point()[1]);
                    moveStatus.setBackgroundColor(Color.GREEN);
                    rotateStatus.setBackgroundColor(Color.TRANSPARENT);
                    graspStatus.setBackgroundColor(Color.TRANSPARENT);
                }
            });
            gestureHandler.setSingleDragX(0);
            gestureHandler.setLongClickX(0);
        }
    }

    private void updateRotation() {
        final float angle = 0.5f*gestureHandler.getAngle()*3.1416f/180f;
        if(!gestureHandler.isDetectingTwoFingerGesture()){
            gestureHandler.setAngle(0);
        }
        if(angle!=0){
            msg = String.format("Rotating Jaw += %.2f ",angle);
            rotationNode.getPublisher_point()[0]=0;
            rotationNode.getPublisher_point()[1]=angle;
            gestureHandler.setDoubleDragX(0);
            rotationNode.publishNow();
            vsNode.setPublisher_bool(false);
            this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    positionImage.setAlpha(0.0f);
                    targetImage.setAlpha(0.0f);

                    moveStatus.setBackgroundColor(Color.TRANSPARENT);
                    rotateStatus.setBackgroundColor(Color.GREEN);
                    graspStatus.setBackgroundColor(Color.TRANSPARENT);
                }
            });
        }

    }

    private void updateGrasping() {
        float grasp = MAX_GRASP*(TwoFingerGestureDetector.MAX_SCALE-gestureHandler.getScale())/(TwoFingerGestureDetector.MAX_SCALE-TwoFingerGestureDetector.MIN_SCALE);
        if(grasp!=graspNode.getPublisher_float()){
            graspNode.setPublisher_float(grasp);
            msg = String.format("Grasping = %.2f ",grasp);
            this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    positionImage.setAlpha(0.0f);
                    targetImage.setAlpha(0.0f);
                    moveStatus.setBackgroundColor(Color.TRANSPARENT);
                    rotateStatus.setBackgroundColor(Color.TRANSPARENT);
                    graspStatus.setBackgroundColor(Color.GREEN);
                }
            });
        }
        graspNode.publishNow();
    }

    private void updateText() {
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                msgText.setText(msg);
                /*if(enableMoveStatus)
                    moveStatus.setBackgroundColor(Color.GREEN);
                else
                    moveStatus.setBackgroundColor(Color.TRANSPARENT);

                if(enableRotateStatus)
                    rotateStatus.setBackgroundColor(Color.GREEN);
                else
                    rotateStatus.setBackgroundColor(Color.TRANSPARENT);

                if(enableGraspStatus)
                    graspStatus.setBackgroundColor(Color.GREEN);
                else
                    graspStatus.setBackgroundColor(Color.TRANSPARENT);
                    */
            }
        });
    }

    private boolean validTarget(float x, float y) {
        if (x < 0 || y < 0)
            return false;
        if (x > imageStream.getDrawable().getIntrinsicWidth() ||  y > imageStream.getDrawable().getIntrinsicHeight())
            return false;
        return true;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        /*
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            float x = event.values[0];
            float y = event.values[1];
            float z = event.values[2];

            z=y-1; //for testing with shield TODO
            y=x-1;

            if(Math.abs(y)>ROLL_THREASHOLD){
                rollText.setAlpha(1.f);
                jawText.setAlpha(.1f);
            }
            if(Math.abs(z)>JAW_THREASHOLD) {
                rollText.setAlpha(.1f);
                jawText.setAlpha(1.f);
            }
        }
        */
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        nodeMain=(NodeMainExecutorService)nodeMainExecutor;
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeMainExecutor.execute(androidNode, nodeConfiguration.setNodeName(androidNode.getName()));
    }

}