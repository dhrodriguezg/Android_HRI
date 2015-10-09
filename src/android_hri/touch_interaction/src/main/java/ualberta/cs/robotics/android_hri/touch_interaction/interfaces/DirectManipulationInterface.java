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
    private static final String NODE_NAME="/android_"+TAG.toLowerCase();
/*
    private static final String STREAMING= "/image_converter/output_video/compressed";
    private static final String STREAMING_MSG = "sensor_msgs/CompressedImage";
    private static final String EMERGENCY_STOP = "/android/emergency_stop";
    private static final String INTERFACE_NUMBER="/android/interface_number";
    private static final String POSITION="/android/position_abs";
    private static final String ROTATION= "/android/rotation_rel";
    private static final String GRASP="/android/grasping_abs";
*/
    private NodeMainExecutorService nodeMain;

    private static final float MAX_GRASP = 2.0f;

    private MultiTouchArea gestureHandler = null;

    private RosImageView<CompressedImage> imageStreamNodeMain;
    private ImageView targetImage;
    private ImageView positionImage;
    private TextView msgText;

    private TextView moveStatus;
    private TextView rotateStatus;
    private TextView graspStatus;

    private AndroidNode androidNode;
    private BooleanTopic emergencyTopic;
    private Int32Topic interfaceNumberTopic;
    private PointTopic positionTopic;
    private PointTopic rotationTopic;
    private Float32Topic graspTopic;



    private String msg="";
    private String moveMsg="";
    private String rotateMsg="";
    private String graspMsg="";
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

        moveMsg=getString(R.string.move_msg) + " (%.4f , %.4f)";
        rotateMsg=getString(R.string.rotate_msg) + " += %.2f";
        graspMsg=getString(R.string.grasp_msg) + " = %.2f";
        targetImage = (ImageView) findViewById(R.id.targetView);
        positionImage = (ImageView) findViewById(R.id.positionView);
        msgText = (TextView) findViewById(R.id.msgTextView);
        moveStatus = (TextView) findViewById(R.id.moveStatus);
        rotateStatus = (TextView) findViewById(R.id.rotateStatus);
        graspStatus = (TextView) findViewById(R.id.graspStatus);

        imageStreamNodeMain = (RosImageView<CompressedImage>) findViewById(R.id.imageViewCenter);

        imageStreamNodeMain.setTopicName(getString(R.string.topic_streaming));
        imageStreamNodeMain.setMessageType(getString(R.string.topic_streaming_msg));
        imageStreamNodeMain.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        imageStreamNodeMain.setScaleType(ImageView.ScaleType.FIT_CENTER);

        gestureHandler = new MultiTouchArea(this, imageStreamNodeMain);
        gestureHandler.enableScaling();
        //gestureHandler.enableDragging();
        gestureHandler.enableScroll(); //single dragging
        gestureHandler.enableRotating();
        gestureHandler.enableOneFingerGestures();
        //Disabled Select target gestures
        //gestureHandler.enableDoubleTap();
        //gestureHandler.enableLongPress();

        positionTopic = new PointTopic();
        positionTopic.publishTo(getString(R.string.topic_positionabs), false, 10);

        graspTopic = new Float32Topic();
        graspTopic.publishTo(getString(R.string.topic_graspingabs), false, 2);
        graspTopic.setPublishingFreq(500);

        rotationTopic = new PointTopic();
        rotationTopic.publishTo(getString(R.string.topic_rotationrel), false, 10);

        interfaceNumberTopic = new Int32Topic();
        interfaceNumberTopic.publishTo(getString(R.string.topic_interfacenumber), true, 0);
        interfaceNumberTopic.setPublishingFreq(100);
        interfaceNumberTopic.setPublisher_int(3);

        emergencyTopic = new BooleanTopic();
        emergencyTopic.publishTo(getString(R.string.topic_emergencystop), true, 0);
        emergencyTopic.setPublishingFreq(100);
        emergencyTopic.setPublisher_bool(true);

        androidNode = new AndroidNode(NODE_NAME);
        androidNode.addTopics(positionTopic, graspTopic, rotationTopic, emergencyTopic, interfaceNumberTopic);
        androidNode.addNodeMain(imageStreamNodeMain);

        senSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        senAccelerometer = senSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        senSensorManager.registerListener(this, senAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);

        ToggleButton emergencyStop = (ToggleButton)findViewById(R.id.emergencyButton) ;
        emergencyStop.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                if (isChecked) {
                    Toast.makeText(getApplicationContext(), getString(R.string.emergency_on_msg), Toast.LENGTH_LONG).show();
                    imageStreamNodeMain.setBackgroundColor(Color.RED);
                    emergencyTopic.setPublisher_bool(false);
                } else {
                    Toast.makeText(getApplicationContext(), getString(R.string.emergency_off_msg), Toast.LENGTH_LONG).show();
                    imageStreamNodeMain.setBackgroundColor(Color.TRANSPARENT);
                    emergencyTopic.setPublisher_bool(true);
                }
            }
        });

        updateGrasping();
        msg="";
        Thread threadGestures = new Thread(){
            public void run(){
                while(running){
                    try {
                        TwoFingerGestureDetector.MAX_RESOLUTION= imageStreamNodeMain.getWidth();
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
        emergencyTopic.setPublisher_bool(true);
        running=true;
    }
    
    @Override
    protected void onPause() {
        emergencyTopic.setPublisher_bool(false);
    	super.onPause();
    }
    
    @Override
    public void onDestroy() {
        emergencyTopic.setPublisher_bool(false);
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

    private void updatePosition() {
        if(gestureHandler.getSingleDragX()>0){

            final float x=gestureHandler.getSingleDragX() - positionImage.getWidth()/2;
            final float y=gestureHandler.getSingleDragY() - positionImage.getHeight()/2;

            float[] positionPoint = new float[]{gestureHandler.getSingleDragX(), gestureHandler.getSingleDragY()};
            float[] positionPixel = new float[2];

            Matrix streamMatrix = new Matrix();
            imageStreamNodeMain.getImageMatrix().invert(streamMatrix);
            streamMatrix.mapPoints(positionPixel, positionPoint);

            if(!validTarget(positionPixel[0],positionPixel[1])){
                gestureHandler.setSingleDragX(0);
                return;
            }

            positionTopic.getPublisher_point()[0] = MainActivity.WORKSPACE_Y_OFFSET - positionPixel[1]*MainActivity.WORKSPACE_HEIGHT/(float) imageStreamNodeMain.getDrawable().getIntrinsicHeight();
            positionTopic.getPublisher_point()[1] = MainActivity.WORKSPACE_X_OFFSET - positionPixel[0]*MainActivity.WORKSPACE_WIDTH/(float) imageStreamNodeMain.getDrawable().getIntrinsicWidth();
            positionTopic.getPublisher_point()[2] = 0;
            positionTopic.publishNow();

            this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    positionImage.setX(x);
                    positionImage.setY(y);
                    positionImage.setAlpha(0.4f);

                    targetImage.setAlpha(0.f);
                    targetImage.setX(0);
                    targetImage.setY(0);

                    msg = String.format(moveMsg, positionTopic.getPublisher_point()[0], positionTopic.getPublisher_point()[1]);
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
            msg = String.format(rotateMsg,angle);
            rotationTopic.getPublisher_point()[0]=0;
            rotationTopic.getPublisher_point()[1]=angle;
            gestureHandler.setDoubleDragX(0);
            rotationTopic.publishNow();
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
        if(grasp!= graspTopic.getPublisher_float()){
            graspTopic.setPublisher_float(grasp);
            msg = String.format(graspMsg,grasp);
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
        graspTopic.publishNow();
    }

    private void updateText() {
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                msgText.setText(msg);
            }
        });
    }

    private boolean validTarget(float x, float y) {
        if (x < 0 || y < 0)
            return false;
        if (x > imageStreamNodeMain.getDrawable().getIntrinsicWidth() ||  y > imageStreamNodeMain.getDrawable().getIntrinsicHeight())
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