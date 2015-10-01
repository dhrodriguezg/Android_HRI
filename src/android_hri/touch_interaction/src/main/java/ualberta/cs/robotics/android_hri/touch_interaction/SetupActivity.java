package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
import android.graphics.Color;
import android.graphics.Matrix;
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
import ualberta.cs.robotics.android_hri.touch_interaction.node.BooleanNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.Int32Node;
import ualberta.cs.robotics.android_hri.touch_interaction.node.PointNode;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.MultiTouchArea;


public class SetupActivity extends RosActivity {

	private static final String TAG = "SetupActivity";
    private static final String STREAMING= "/image_converter/output_video/compressed";
    private static final String STREAMING_MSG = "sensor_msgs/CompressedImage";
    private static final String EMERGENCY_STOP = "/android/emergency_stop";
    private static final String ENABLE_VS = "/android/enable_vs";
    private static final String TRACKER_POINT ="/android/tracker_point";
    private static final String TARGET_POINT="/android/target_point";
    private static final String INTERFACE_NUMBER="/android/interface_number";
    private static final String SETUP_ON="/android/setup/on";
    private static final String SETUP_OFF="/android/setup/off";
    private static final String POS1_STATE="/android/setup/pos1_state";
    private static final String POS2_STATE="/android/setup/pos2_state";
    private static final String GRASP_STATE="/android/setup/grasp_state";
    private static final String SPREAD_STATE="/android/setup/spread_state";
    private final int DISABLED = Color.RED;
    private final int ENABLED = Color.GREEN;
    private final int TRANSITION = Color.rgb(255,195,77); //orange

    private NodeMainExecutorService nodeMain;

    private static final boolean debug = true;
    private MultiTouchArea dragHandler = null;

    private RosImageView<CompressedImage> imageStream;
    private boolean running = true;
    private boolean updateCenter=false;

    private float correctionX =0.f;
    private float correctionY =0.f;
    private float scaleCorrection=1.f;
    private float scaleTemp=1.f;
    private float traslationTempX=0.f;
    private float traslationTempY=0.f;

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
    private Int32Node interfaceNumberNode;
    private BooleanNode setupONNode;
    private BooleanNode setupOFFNode;
    private BooleanNode emergencyNode;
    private BooleanNode vsNode;

    private boolean firstRun=true;
    private static final int MAX_TRACKERS=2;
    private int trackerNumber=0;

    public SetupActivity() {
        super(TAG, TAG, URI.create(MainActivity.ROS_MASTER));
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {

    	Intent intent = getIntent();
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_setup);

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

        setupONNode = new BooleanNode();
        setupONNode.publishTo(SETUP_ON, false, 100);

        setupOFFNode = new BooleanNode();
        setupOFFNode.publishTo(SETUP_OFF, false, 100);

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

        interfaceNumberNode = new Int32Node();
        interfaceNumberNode.publishTo(INTERFACE_NUMBER, true, 0);
        interfaceNumberNode.setPublishFreq(100);
        interfaceNumberNode.setPublish_int(-1);

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
                    setupONNode.setPublish_bool(true);
                    setupONNode.publishNow();
                    buttonOFF.setEnabled(false);
                    statusPos1_ON.setBackgroundColor(DISABLED);
                    statusPos2_ON.setBackgroundColor(DISABLED);
                    statusGrasp_ON.setBackgroundColor(DISABLED);
                    statusSpread_ON.setBackgroundColor(DISABLED);
                } else {
                    setupONNode.setPublish_bool(false);
                    setupONNode.publishNow();
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
                    setupOFFNode.setPublish_bool(true);
                    setupOFFNode.publishNow();
                    buttonON.setEnabled(false);
                    statusPos1_OFF.setBackgroundColor(DISABLED);
                    statusPos2_OFF.setBackgroundColor(DISABLED);
                    statusGrasp_OFF.setBackgroundColor(DISABLED);
                    statusSpread_OFF.setBackgroundColor(DISABLED);
                } else {
                    setupOFFNode.setPublish_bool(false);
                    setupOFFNode.publishNow();
                    buttonON.setEnabled(true);
                }
            }
        });

        ToggleButton emergencyStop = (ToggleButton)findViewById(R.id.emergencyButton) ;
        emergencyStop.setOnCheckedChangeListener( new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                if(isChecked){
                    Toast.makeText(getApplicationContext(), "EMERGENCY STOP ACTIVATED!", Toast.LENGTH_LONG).show();
                    imageStream.setBackgroundColor(Color.RED);
                    emergencyNode.setPublish_bool(false);
                }else{
                    Toast.makeText(getApplicationContext(), "EMERGENCY STOP DEACTIVATED!", Toast.LENGTH_LONG).show();
                    imageStream.setBackgroundColor(Color.TRANSPARENT);
                    emergencyNode.setPublish_bool(true);
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
                        updateStatuses();
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

    public void updateStatuses() {

        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {

                if (buttonON.isChecked()) {
                    if (pos1State.hasReceivedMsg()) {
                        pos1State.setHasReceivedMsg(false);
                        switch (pos1State.getSubcribe_int()) {
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

                    if (pos2State.hasReceivedMsg()) {
                        pos2State.setHasReceivedMsg(false);
                        switch (pos2State.getSubcribe_int()) {
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

                    if (spreadState.hasReceivedMsg()) {
                        spreadState.setHasReceivedMsg(false);
                        switch (spreadState.getSubcribe_int()) {
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

                    if (graspState.hasReceivedMsg()) {
                        graspState.setHasReceivedMsg(false);
                        switch (graspState.getSubcribe_int()) {
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

                } else if (buttonOFF.isChecked()) {
                    //logic for OFF
                    if (pos1State.hasReceivedMsg()) {
                        pos1State.setHasReceivedMsg(false);
                        switch (pos1State.getSubcribe_int()) {
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

                    if (pos2State.hasReceivedMsg()) {
                        pos2State.setHasReceivedMsg(false);
                        switch (pos2State.getSubcribe_int()) {
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

                    if (spreadState.hasReceivedMsg()) {
                        spreadState.setHasReceivedMsg(false);
                        switch (spreadState.getSubcribe_int()) {
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

                    if (graspState.hasReceivedMsg()) {
                        graspState.setHasReceivedMsg(false);
                        switch (graspState.getSubcribe_int()) {
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
        nodeMain=(NodeMainExecutorService)nodeMainExecutor;
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
        nodeMainExecutor.execute(setupONNode, nodeConfiguration.setNodeName(SETUP_ON));
        nodeMainExecutor.execute(setupOFFNode, nodeConfiguration.setNodeName(SETUP_OFF));
        nodeMainExecutor.execute(interfaceNumberNode, nodeConfiguration.setNodeName(INTERFACE_NUMBER));
    }

}