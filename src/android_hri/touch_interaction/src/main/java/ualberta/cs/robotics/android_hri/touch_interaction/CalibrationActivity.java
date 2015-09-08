package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
import android.graphics.Color;
import android.graphics.Matrix;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.ImageView;

import android.widget.Switch;
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
import ualberta.cs.robotics.android_hri.touch_interaction.node.Int32Node;
import ualberta.cs.robotics.android_hri.touch_interaction.node.PointNode;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.MultiTouchArea;


public class CalibrationActivity extends RosActivity {

	private static final String TAG = "CalibrationActivity";
    private static final String STREAMING= "/image_converter/output_video/compressed";
    private static final String STREAMING_MSG = "sensor_msgs/CompressedImage";
    private static final String EMERGENCY_STOP = "/android/emergency_stop";
    private static final String ENABLE_VS = "/android/enable_vs";
    private static final String TRACKER_POINT ="/android/tracker_point";
    private static final String TARGET_POINT="/android/target_point";
    private static final String SETUP_ON="/android/setup/on";
    private static final String SETUP_OFF="/android/setup/off";
    private static final String POS1_STATE="/android/setup/pos1State";
    private static final String POS2_STATE="/android/setup/pos2State";
    private static final String GRASP_STATE="/android/setup/graspState";
    private static final String SPREAD_STATE="/android/setup/spreadState";
    private final int DISABLED = Color.RED;
    private final int ENABLED = Color.GREEN;
    private final int TRANSITION = Color.rgb(255,195,77); //orange

    private NodeMainExecutor nodeMain;

    private static final boolean debug = false;
    private MultiTouchArea dragHandler = null;

    private RosImageView<CompressedImage> imageStream;
    private Button confirmButton = null;
    private Switch startSwitch = null;
    private boolean running = true;
    private boolean updateCenter=false;

    private float correctionX =0.f;
    private float correctionY =0.f;
    private float scaleCorrection=1.f;
    private float scaleTemp=1.f;
    private float traslationTempX=0.f;
    private float traslationTempY=0.f;
    private float tracker_x;
    private float tracker_y;

    private ToggleButton buttonON;
    private TextView statusPos1_ON;
    private TextView statusPos2_ON;
    private TextView statusSpread_ON;
    private TextView statusGrasp_ON;

    private ToggleButton buttonOFF;
    private TextView statusPos1_OFF;
    private TextView statusPos2_OFF;
    private TextView statusSpread_OFF;
    private TextView statusGrasp_OFF;

    private PointNode trackerPointNode;
    private PointNode targetPointNode;
    private Int32Node pos1State;
    private Int32Node pos2State;
    private Int32Node graspState;
    private Int32Node spreadState;
    private BooleanNode setupON;
    private BooleanNode setupOFF;
    private BooleanNode emergencyNode;
    private BooleanNode vsNode;

    private boolean firstRun=true;
    private static final int MAX_TRACKERS=2;
    private int trackerNumber=0;

    public CalibrationActivity() {
        super(TAG, TAG, URI.create(MainActivity.ROS_MASTER));
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {

    	Intent intent = getIntent();
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_calibration);

        statusPos1_ON = (TextView) findViewById(R.id.statusPos1_ON);
        statusPos2_ON = (TextView) findViewById(R.id.statusPos2_ON);
        statusSpread_ON = (TextView) findViewById(R.id.statusSpread_ON);
        statusGrasp_ON = (TextView) findViewById(R.id.statusGrasp_ON);
        statusPos1_OFF = (TextView) findViewById(R.id.statusPos1_OFF);
        statusPos2_OFF = (TextView) findViewById(R.id.statusPos2_OFF);
        statusSpread_OFF = (TextView) findViewById(R.id.statusSpread_OFF);
        statusGrasp_OFF = (TextView) findViewById(R.id.statusGrasp_OFF);

        imageStream = (RosImageView<CompressedImage>) findViewById(R.id.streamCalibration);
        if(debug){
            /**
             cd /opt/ros/indigo/share/openni2_launch/launch
             roslaunch openni2.launch
             rosrun image_view image_view image:=/camera/rgb/image_raw

             cd /home/dhrodriguezg/catkin_ws/launch_files
             roslaunch usb_camera.launch

             rostopic echo /android/joystickPos/cmd_vel
            */
            imageStream.setTopicName("/usb_cam/image_raw/compressed");

        }else
            imageStream.setTopicName(STREAMING);
        imageStream.setMessageType(STREAMING_MSG);

        setupON = new BooleanNode();
        setupON.publishTo(SETUP_ON, false, 100);

        setupOFF = new BooleanNode();
        setupOFF.publishTo(SETUP_OFF, false, 100);

        pos1State = new Int32Node();
        pos1State.subscribeTo(POS1_STATE);
        pos2State = new Int32Node();
        pos2State.subscribeTo(POS2_STATE);
        graspState = new Int32Node();
        graspState.subscribeTo(GRASP_STATE);
        spreadState = new Int32Node();
        spreadState.subscribeTo(SPREAD_STATE);

        trackerPointNode = new PointNode();
        trackerPointNode.publishTo(TRACKER_POINT, false, 10);

        targetPointNode = new PointNode();
        targetPointNode.publishTo(TARGET_POINT, false, 10);

        emergencyNode = new BooleanNode();
        emergencyNode.publishTo(EMERGENCY_STOP, true, 0);
        emergencyNode.setPublishFreq(100);
        emergencyNode.setPublish_bool(true);

        vsNode = new BooleanNode();
        vsNode.publishTo(ENABLE_VS, false, 100);
        vsNode.setPublish_bool(true);

        imageStream.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        imageStream.setScaleType(ImageView.ScaleType.MATRIX);


        buttonON = (ToggleButton)findViewById(R.id.buttonON) ;
        buttonON.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                if (isChecked) {
                    Toast.makeText(getApplicationContext(), "Going to Task position", Toast.LENGTH_LONG).show();
                    setupON.setPublish_bool(true);
                    setupON.publishNow();
                    buttonOFF.setEnabled(false);
                } else {
                    setupON.setPublish_bool(false);
                    setupON.publishNow();
                    buttonOFF.setEnabled(true);
                }
            }
        });

        buttonOFF = (ToggleButton)findViewById(R.id.buttonOFF);
        buttonOFF.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                if (isChecked) {
                    Toast.makeText(getApplicationContext(), "Going to Initial position", Toast.LENGTH_LONG).show();
                    setupOFF.setPublish_bool(true);
                    setupOFF.publishNow();
                    buttonON.setEnabled(false);
                } else {
                    setupOFF.setPublish_bool(false);
                    setupOFF.publishNow();
                    buttonON.setEnabled(true);
                }
            }
        });


        confirmButton = (Button) findViewById(R.id.confirmButton);
        confirmButton.setOnClickListener( new View.OnClickListener() {
            @Override
            public void onClick( View v ) {
                vsNode.publishNow();
                switch (trackerNumber){
                    case 0:
                        trackerPointNode.getPublish_point()[0]=tracker_x;
                        trackerPointNode.getPublish_point()[1]=tracker_y;
                        trackerPointNode.getPublish_point()[2]=0;
                        trackerPointNode.publishNow();
                        Toast.makeText(getApplicationContext(), "Tracker selected at: "+(int)tracker_x + " , " + (int)tracker_y, Toast.LENGTH_LONG).show();
                        break;
                    case 1:
                        targetPointNode.getPublish_point()[0]=tracker_x;
                        targetPointNode.getPublish_point()[1]=tracker_y;
                        targetPointNode.getPublish_point()[2]=0;
                        targetPointNode.publishNow();
                        Toast.makeText(getApplicationContext(), "Target selected at: "+(int)tracker_x + " , " + (int)tracker_y, Toast.LENGTH_LONG).show();
                        break;
                    default:
                        Toast.makeText(getApplicationContext(), "You can't add more markers, MAX: "+MAX_TRACKERS, Toast.LENGTH_LONG).show();
                        break;
                }
                trackerNumber++;
            }
        } );
        startSwitch = (Switch) findViewById(R.id.startSwitch);
        startSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                confirmButton.setEnabled(isChecked);
                if (isChecked){
                    Toast.makeText(getApplicationContext(), "Starting the calibration, select tracker and target...", Toast.LENGTH_LONG).show();
                    trackerNumber=0;
                }
            }
        });

        dragHandler = new MultiTouchArea(this, imageStream);
        dragHandler.enableScaling();
        dragHandler.enableOneFingerGestures();
        dragHandler.enableScroll();

        Thread threadTarget = new Thread(){
            public void run(){
                while(running){
                    try {
                        updateTarget();
                        Thread.sleep(20);
                    } catch (InterruptedException e) {
                        e.getStackTrace();
                    }
                }
            }
        };
        threadTarget.start();
    }

    @Override
    public void onResume() {
        super.onResume();
        emergencyNode.setPublish_bool(true);
        vsNode.setPublish_bool(true);
        running=true;
    }
    
    @Override
    protected void onPause() {
    	super.onPause();
        emergencyNode.setPublish_bool(false);
        vsNode.setPublish_bool(false);
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

    public void updateStatuses() {

        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {

                if(buttonON.isChecked()){

                    if(pos1State.hasReceivedMsg()){
                        pos1State.setHasReceivedMsg(false);
                        switch (pos1State.getSubcribe_int()){
                            case -1:
                                statusPos1_ON.setBackgroundColor(DISABLED);
                                break;
                            case 0:
                                statusPos1_ON.setBackgroundColor(TRANSITION);
                                break;
                            case 1:
                                statusPos1_ON.setBackgroundColor(ENABLED);
                                break;
                            default:

                                break;
                        }
                    }

                    if(pos2State.hasReceivedMsg()){
                        pos2State.setHasReceivedMsg(false);
                        switch (pos2State.getSubcribe_int()){
                            case -1:
                                statusPos2_ON.setBackgroundColor(DISABLED);
                                break;
                            case 0:
                                statusPos2_ON.setBackgroundColor(TRANSITION);
                                break;
                            case 1:
                                statusPos2_ON.setBackgroundColor(ENABLED);
                                break;
                            default:

                                break;
                        }
                    }

                    if(spreadState.hasReceivedMsg()){
                        spreadState.setHasReceivedMsg(false);
                        switch (spreadState.getSubcribe_int()){
                            case -1:
                                statusSpread_ON.setBackgroundColor(DISABLED);
                                break;
                            case 0:
                                statusSpread_ON.setBackgroundColor(TRANSITION);
                                break;
                            case 1:
                                statusSpread_ON.setBackgroundColor(ENABLED);
                                break;
                            default:

                                break;
                        }
                    }

                    if(graspState.hasReceivedMsg()){
                        graspState.setHasReceivedMsg(false);
                        switch (graspState.getSubcribe_int()){
                            case -1:
                                statusGrasp_ON.setBackgroundColor(ENABLED);
                                break;
                            case 0:
                                statusGrasp_ON.setBackgroundColor(TRANSITION);
                                break;
                            case 1:
                                statusGrasp_ON.setBackgroundColor(DISABLED);
                                break;
                            default:
                                break;
                        }
                    }

                }else if(buttonOFF.isChecked()){
                    //logic for OFF
                    if(pos1State.hasReceivedMsg()){
                        pos1State.setHasReceivedMsg(false);
                        switch (pos1State.getSubcribe_int()){
                            case -1:
                                statusPos1_OFF.setBackgroundColor(DISABLED);
                                break;
                            case 0:
                                statusPos1_OFF.setBackgroundColor(TRANSITION);
                                break;
                            case 1:
                                statusPos1_OFF.setBackgroundColor(ENABLED);
                                break;
                            default:

                                break;
                        }
                    }

                    if(pos2State.hasReceivedMsg()){
                        pos2State.setHasReceivedMsg(false);
                        switch (pos2State.getSubcribe_int()){
                            case -1:
                                statusPos2_OFF.setBackgroundColor(DISABLED);
                                break;
                            case 0:
                                statusPos2_OFF.setBackgroundColor(TRANSITION);
                                break;
                            case 1:
                                statusPos2_OFF.setBackgroundColor(ENABLED);
                                break;
                            default:

                                break;
                        }
                    }

                    if(spreadState.hasReceivedMsg()){
                        spreadState.setHasReceivedMsg(false);
                        switch (spreadState.getSubcribe_int()){
                            case -1:
                                statusSpread_OFF.setBackgroundColor(ENABLED);
                                break;
                            case 0:
                                statusSpread_OFF.setBackgroundColor(TRANSITION);
                                break;
                            case 1:
                                statusSpread_OFF.setBackgroundColor(DISABLED);
                                break;
                            default:
                                break;
                        }
                    }

                    if(graspState.hasReceivedMsg()){
                        graspState.setHasReceivedMsg(false);
                        switch (graspState.getSubcribe_int()){
                            case -1:
                                statusGrasp_OFF.setBackgroundColor(DISABLED);
                                break;
                            case 0:
                                statusGrasp_OFF.setBackgroundColor(TRANSITION);
                                break;
                            case 1:
                                statusGrasp_OFF.setBackgroundColor(ENABLED);
                                break;
                            default:

                                break;
                        }
                    }
                }
            }
        });

    }

    public void updateTarget() {

        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {

                float viewWidth=imageStream.getWidth();
                float viewHeight=imageStream.getHeight();
                float streamWidth=imageStream.getDrawable().getIntrinsicWidth();
                float streamHeight=imageStream.getDrawable().getIntrinsicHeight();

                float focusX=viewWidth / 2;
                float focusY=viewHeight / 2;

                if (firstRun) {
                    float scaleX = viewWidth / streamWidth;
                    float scaleY = viewHeight / streamHeight;
                    dragHandler.setScaleFocusX(focusX);
                    dragHandler.setScaleFocusY(focusY);
                    scaleCorrection = Math.min(scaleX, scaleY);
                    scaleTemp = scaleCorrection;
                    if (dragHandler.getScale() != 1.f || dragHandler.getSingleDragX()!=0)
                        firstRun = false;
                }

                float scale = dragHandler.getScale()*scaleCorrection;

                float imageScaledWidth = scale*streamWidth;
                float imageScaledWHeight = scale*streamHeight;

                float imageScaledCenteredX = (viewWidth - imageScaledWidth)/ 2;
                float imageScaledCenteredY = (viewHeight - imageScaledWHeight)/ 2;

                float imageTraslationX = dragHandler.getSingleDragX()-dragHandler.getSingleIDragX();
                float imageTraslationY = dragHandler.getSingleDragY()-dragHandler.getSingleIDragY();

                float finalScaledCenteredX=imageScaledCenteredX + imageTraslationX + correctionX;
                float finalScaledCenteredY=imageScaledCenteredY + imageTraslationY + correctionY;

                if (scaleTemp==scale) {
                    traslationTempX=finalScaledCenteredX;
                    traslationTempY=finalScaledCenteredY;

                }

                float[] targetPoint = new float[2];
                float[] targetPixel = new float[2];
                float[] errorPoint = new float[2];
                targetPoint[0] = focusX;
                targetPoint[1] = focusY;

                Matrix previowsTMatrix = new Matrix();
                Matrix previowsInverseMatrix = new Matrix();
                previowsTMatrix.setScale(scaleTemp, scaleTemp);
                previowsTMatrix.postTranslate(traslationTempX, traslationTempY);
                previowsTMatrix.invert(previowsInverseMatrix);

                previowsInverseMatrix.mapPoints(targetPixel, targetPoint);

                Matrix postTMatrix = new Matrix();
                postTMatrix.setScale(scale, scale);
                postTMatrix.postTranslate(finalScaledCenteredX, finalScaledCenteredY);
                postTMatrix.mapPoints(errorPoint, targetPixel);
                float scaleCorrectionX = focusX - errorPoint[0];
                float scaleCorrectionY = focusY - errorPoint[1];

                Log.d(TAG, String.format("Centers [ %.4f %.4f %.4f %.4f ]", errorPoint[0], errorPoint[1], targetPixel[0], targetPixel[1]));
                tracker_x=targetPixel[0];
                tracker_y=targetPixel[1];

                Matrix finalTMatrix = new Matrix();
                finalTMatrix.setScale(scale, scale);
                finalTMatrix.postTranslate(finalScaledCenteredX + scaleCorrectionX, finalScaledCenteredY + scaleCorrectionY);

                imageStream.setImageMatrix(finalTMatrix);

                if(dragHandler.isDetectingOneFingerGesture()){
                    updateCenter=false;
                }else{
                    if(!updateCenter){
                        correctionX += imageTraslationX + scaleCorrectionX;
                        correctionY += imageTraslationY + scaleCorrectionY;
                        dragHandler.resetValuesOnRelease();
                        scaleTemp=scale;
                    }
                    updateCenter=true;
                }

            }
        });
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        nodeMain=nodeMainExecutor;
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeMainExecutor.execute(imageStream, nodeConfiguration.setNodeName(STREAMING + "sub"));
        nodeMainExecutor.execute(pos1State, nodeConfiguration.setNodeName(POS1_STATE + "sub"));
        nodeMainExecutor.execute(pos2State, nodeConfiguration.setNodeName(POS2_STATE + "sub"));
        nodeMainExecutor.execute(graspState, nodeConfiguration.setNodeName(GRASP_STATE + "sub"));
        nodeMainExecutor.execute(spreadState, nodeConfiguration.setNodeName(SPREAD_STATE + "sub"));

        nodeMainExecutor.execute(trackerPointNode, nodeConfiguration.setNodeName(TRACKER_POINT));
        nodeMainExecutor.execute(targetPointNode, nodeConfiguration.setNodeName(TARGET_POINT));
        nodeMainExecutor.execute(emergencyNode, nodeConfiguration.setNodeName(EMERGENCY_STOP));
        nodeMainExecutor.execute(vsNode, nodeConfiguration.setNodeName(ENABLE_VS));
        nodeMainExecutor.execute(setupON, nodeConfiguration.setNodeName(SETUP_ON));
        nodeMainExecutor.execute(setupOFF, nodeConfiguration.setNodeName(SETUP_OFF));
    }

}
