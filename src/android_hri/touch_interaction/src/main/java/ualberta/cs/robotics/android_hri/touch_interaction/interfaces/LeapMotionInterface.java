package ualberta.cs.robotics.android_hri.touch_interaction.interfaces;

import android.content.Intent;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.drawable.ColorDrawable;
import android.os.Bundle;
import android.view.MenuItem;
import android.view.WindowManager;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.leapmotion.leap.Controller;
import com.leapmotion.leap.Vector;

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
import ualberta.cs.robotics.android_hri.touch_interaction.node.BooleanNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.Float32Node;
import ualberta.cs.robotics.android_hri.touch_interaction.node.Int32Node;
import ualberta.cs.robotics.android_hri.touch_interaction.node.PointNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.StringNode;
import ualberta.cs.robotics.android_hri.touch_interaction.utils.LeapMotionListener;

public class LeapMotionInterface extends RosActivity implements LeapMotionListener.LeapMotionFrameListener{

	private static final String TAG = "LeapMotionInterface";
    private static final String STREAMING= "/image_converter/output_video/compressed";
    private static final String STREAMING_MSG = "sensor_msgs/CompressedImage";
    private static final String EMERGENCY_STOP = "/android/emergency_stop";
    private static final String INTERFACE_NUMBER="/android/interface_number";
    private static final String ENABLE_VS = "/android/enable_vs";
    private static final String STRING_LOG = "/android/log";
    private static final String POSITION="/android/position_abs";
    private static final String TARGET_POINT="/android/target_point";
    private static final String ROTATION= "/android/rotation_abs";
    private static final String GRASP="/android/grasping_abs";

    private final int DISABLED = Color.RED;
    private final int ENABLED = Color.GREEN;
    private final int TRANSITION = Color.rgb(255,195,77); //orange
    private final int MAX_TASK_COUNTER = 90;

    private int confirmCounter;
    private boolean isConfirm=false;
    private boolean isEnable=true;

    private int lastTask = -1;
    private int currentTask = -1;

    private NodeMainExecutorService nodeMain;
    private Controller mController;
    private LeapMotionListener mLeapMotionListener;

    private ToggleButton emergencyStop;

    private static final float MAX_GRASP = 2.0f;

    private RosImageView<CompressedImage> imageStream;
    private TextView msgText;
    private TextView statusText;
    private TextView trackingText;

    private PointNode positionNode;
    private Float32Node graspNode;
    private Int32Node interfaceNumberNode;
    private PointNode rotationNode;
    private StringNode stringNode;
    private BooleanNode emergencyNode;
    private BooleanNode vsNode;

    private Switch rightHanded;
    private CheckBox showLog;
    private CheckBox showHands;
    private ImageView targetMove;

    //left hand
    private ImageView leftHand;
    private ImageView leftIndex;
    private ImageView leftMiddle;
    private ImageView leftRing;
    private ImageView leftPinky;
    private ImageView leftThumb;

    //right hand
    private ImageView rightHand;
    private ImageView rightIndex;
    private ImageView rightMiddle;
    private ImageView rightRing;
    private ImageView rightPinky;
    private ImageView rightThumb;

    private String msg="";
    private boolean running = true;
    private boolean debug = true;

    public LeapMotionInterface() {
        super(TAG, TAG, URI.create(MainActivity.ROS_MASTER));
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {

    	Intent intent = getIntent();
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.interface_leapmotion);

        msgText = (TextView) findViewById(R.id.msgTextView);

        targetMove = (ImageView) findViewById(R.id.imageTarget);
        leftHand = (ImageView) findViewById(R.id.leftHand);
        leftIndex = (ImageView) findViewById(R.id.leftIndexFinger);
        leftMiddle = (ImageView) findViewById(R.id.leftMiddleFinger);
        leftRing = (ImageView) findViewById(R.id.leftRingFinger);
        leftPinky = (ImageView) findViewById(R.id.leftPinkyFinger);
        leftThumb = (ImageView) findViewById(R.id.leftThumbFinger);
        rightHand = (ImageView) findViewById(R.id.rightHand);
        rightIndex = (ImageView) findViewById(R.id.rightIndexFinger);
        rightMiddle = (ImageView) findViewById(R.id.rightMiddleFinger);
        rightRing = (ImageView) findViewById(R.id.rightRingFinger);
        rightPinky = (ImageView) findViewById(R.id.rightPinkyFinger);
        rightThumb = (ImageView) findViewById(R.id.rightThumbFinger);


        showLog = (CheckBox) findViewById(R.id.showLog);

        imageStream = (RosImageView<CompressedImage>) findViewById(R.id.imageViewCenter);
        if(debug)
            imageStream.setTopicName("/usb_cam/image_raw/compressed");
        else {
            imageStream.setTopicName(STREAMING);
            msgText.setAlpha(0f);
        }
        imageStream.setMessageType(STREAMING_MSG);
        imageStream.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        imageStream.setScaleType(ImageView.ScaleType.FIT_CENTER);

        positionNode = new PointNode();
        positionNode.publishTo(POSITION, false, 10);

        graspNode = new Float32Node();
        graspNode.publishTo(GRASP, false, 2);
        graspNode.setPublishFreq(500);

        rotationNode = new PointNode();
        rotationNode.publishTo(ROTATION, false, 10);

        interfaceNumberNode = new Int32Node();
        interfaceNumberNode.publishTo(INTERFACE_NUMBER, true, 0);
        interfaceNumberNode.setPublishFreq(100);
        interfaceNumberNode.setPublish_int(5);

        emergencyNode = new BooleanNode();
        emergencyNode.publishTo(EMERGENCY_STOP, true, 0);
        emergencyNode.setPublishFreq(100);
        emergencyNode.setPublish_bool(true);

        stringNode = new StringNode();
        stringNode.publishTo(STRING_LOG, false, 10);
        stringNode.setPublishFreq(100);

        vsNode = new BooleanNode();
        vsNode.publishTo(ENABLE_VS, true, 0);
        vsNode.setPublish_bool(false);

        mController = new Controller();
        mLeapMotionListener = new LeapMotionListener(this, this);
        mController.addListener(mLeapMotionListener);

        statusText = (TextView)findViewById(R.id.statusView);
        trackingText = (TextView) findViewById(R.id.statusTracking);

        emergencyStop = (ToggleButton)findViewById(R.id.emergencyButton);
        emergencyStop.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                if (isChecked) {
                    Toast.makeText(getApplicationContext(), "EMERGENCY STOP ACTIVATED!", Toast.LENGTH_LONG).show();
                    imageStream.setBackgroundColor(Color.RED);
                    emergencyNode.setPublish_bool(false);
                } else {
                    Toast.makeText(getApplicationContext(), "EMERGENCY STOP DEACTIVATED!", Toast.LENGTH_LONG).show();
                    imageStream.setBackgroundColor(Color.TRANSPARENT);
                    emergencyNode.setPublish_bool(true);
                }
            }
        });

        showHands = (CheckBox) findViewById(R.id.showHands);
        showHands.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                if(!isChecked){
                    hideLeftHand();
                    hideRightHand();
                }
            }
        });

        rightHanded = (Switch) findViewById(R.id.rightHanded);
        rightHanded.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                mLeapMotionListener.setRightHanded(isChecked);
            }
        });
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
        nodeMain.forceShutdown();
        running=false;
        super.onDestroy();
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onHands(final boolean hands) {
        runOnUiThread(new Runnable() {
            public void run() {
                if (hands) {
                    statusText.setText("OK!");
                    statusText.setBackgroundColor(ENABLED);
                } else {
                    statusText.setText("No Hands!");
                    statusText.setBackgroundColor(DISABLED);
                }
            }
        });
    }

    @Override
    public void onSelect(float x, float y, float z) {
        if(!isConfirm)
            return;
    }

    @Override
    public void onMove(float x, float y, float z) {
        if(!isConfirm)
            return;

        vsNode.setPublish_bool(true);
        positionNode.getPublish_point()[0] = MainActivity.WORKSPACE_Y_OFFSET - z*MainActivity.WORKSPACE_HEIGHT;
        positionNode.getPublish_point()[1] = MainActivity.WORKSPACE_X_OFFSET - x*MainActivity.WORKSPACE_WIDTH;
        positionNode.publishNow();

        float[] point = calculatePoint(imageStream.getDrawable().getIntrinsicWidth()*x, imageStream.getDrawable().getIntrinsicHeight()*z);
        final float xm = point[0] - targetMove.getWidth()/2;
        final float ym = point[1] - targetMove.getHeight()/2;
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                targetMove.setX(xm);
                targetMove.setY(ym);
                targetMove.setAlpha(0.4f);
            }
        });
    }

    @Override
    public void onRotate(float x, float y, float z) {
        if(!isConfirm)
            return;
        vsNode.setPublish_bool(false);
        rotationNode.getPublish_point()[0]=y+1.57f;
        rotationNode.getPublish_point()[1]=z;
        rotationNode.publishNow();
    }

    @Override
    public void onGrasping(float g) {
        if(!isConfirm)
            return;
        float grasp = (1f-g)*1.75f;
        graspNode.setPublish_float(grasp);
        graspNode.publishNow();
    }

    @Override
    public void onTask(final int task) {
        if(!isEnable)
            return;
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                switch (task) {
                    case -1: // nonvalid gesture (reset): unused
                        break;
                    case 0: // all fingers closed (reset): unused
                        break;

                    case 1: //move: unused
                        break;
                    case 2: //rotate: unused
                        break;
                    case 3: //grasp: unused
                        break;
                    case 4: //AAAAAALLLLL: unused
                        break;
                    case 5: // all fingers opened (no task): unused
                        break;
                    case 6: //confirm
                        if (confirmCounter > MAX_TASK_COUNTER) {
                            confirmCounter = 0;
                            isConfirm = !isConfirm;
                            lastTask = task;
                            if (isConfirm) {
                                trackingText.setBackgroundColor(ENABLED);
                                Toast.makeText(getApplicationContext(), "Tracking activated", Toast.LENGTH_LONG).show();
                            } else {
                                trackingText.setBackgroundColor(DISABLED);
                                Toast.makeText(getApplicationContext(), "Tracking deactivated", Toast.LENGTH_LONG).show();
                            }
                            return;
                        }
                        confirmCounter++;
                        break;
                    default:
                        break;
                }
            }
        });

    }

    private void resetTasks(){
        isConfirm=false;
        imageStream.setBackgroundColor(Color.TRANSPARENT);
    }

    @Override
    public void onUpdateMsg(final String msg) {
        stringNode.setPublish_string(msg);
        stringNode.publishNow();

        runOnUiThread(new Runnable() {
            public void run() {
                if (!showLog.isChecked()) {
                    msgText.setAlpha(0.0f);
                    return;
                }
                msgText.setText(msg + "\n");
                msgText.setAlpha(0.5f);
            }
        });
    }

    private void hideLeftHand(){
        leftHand.setAlpha(0.0f);
        leftIndex.setAlpha(0.0f);
        leftMiddle.setAlpha(0.0f);
        leftRing.setAlpha(0.0f);
        leftPinky.setAlpha(0.0f);
        leftThumb.setAlpha(0.0f);
    }

    private void hideRightHand(){
        rightHand.setAlpha(0.0f);
        rightIndex.setAlpha(0.0f);
        rightMiddle.setAlpha(0.0f);
        rightRing.setAlpha(0.0f);
        rightPinky.setAlpha(0.0f);
        rightThumb.setAlpha(0.0f);
    }

    @Override
    public void onMoveLeftHand(final Vector[] positions) {
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if(positions!=null && positions[0].getY()>0.8f){
                    isEnable=false;
                    resetTasks();
                }else{
                    isEnable=true;
                }
                if(!showHands.isChecked() || positions==null){
                    hideLeftHand();
                    return;
                }
                int counter=0;
                for(Vector position : positions){
                    float[] point = calculatePoint(imageStream.getDrawable().getIntrinsicWidth() * position.getX() , imageStream.getDrawable().getIntrinsicHeight() * position.getZ() );
                    switch (counter){
                        case 0: //palm
                            leftHand.setX(point[0] - leftHand.getWidth() / 2);
                            leftHand.setY(point[1] - leftHand.getHeight() / 2);
                            leftHand.setAlpha(0.4f);
                            break;
                        case 1: //index
                            leftIndex.setX(point[0] - leftIndex.getWidth()/2);
                            leftIndex.setY(point[1] - leftIndex.getHeight()/2);
                            leftIndex.setAlpha(0.4f);
                            break;
                        case 2: //middle
                            leftMiddle.setX(point[0] - leftMiddle.getWidth()/2);
                            leftMiddle.setY(point[1] - leftMiddle.getHeight()/2);
                            leftMiddle.setAlpha(0.4f);
                            break;
                        case 3: //ring
                            leftRing.setX(point[0] - leftRing.getWidth()/2);
                            leftRing.setY(point[1] - leftRing.getHeight()/2);
                            leftRing.setAlpha(0.4f);
                            break;
                        case 4: //pinky
                            leftPinky.setX(point[0] - leftPinky.getWidth()/2);
                            leftPinky.setY(point[1] - leftPinky.getHeight()/2);
                            leftPinky.setAlpha(0.4f);
                            break;
                        case 5: //thumb
                            leftThumb.setX(point[0] - leftThumb.getWidth()/2);
                            leftThumb.setY(point[1] - leftThumb.getHeight()/2);
                            leftThumb.setAlpha(0.4f);
                            break;
                        default:
                            break;
                    }
                    counter++;
                }
            }
        });
    }

    @Override
    public void onMoveRightHand(final Vector[] positions) {

        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if(!showHands.isChecked()  || positions==null){
                    hideRightHand();
                    return;
                }
                int counter=0;
                for(Vector position : positions){
                    float[] point = calculatePoint(imageStream.getDrawable().getIntrinsicWidth() * position.getX() , imageStream.getDrawable().getIntrinsicHeight() * position.getZ() );
                    switch (counter){
                        case 0: //palm
                            rightHand.setX(point[0] - rightHand.getWidth()/2);
                            rightHand.setY(point[1] - rightHand.getHeight()/2);
                            rightHand.setAlpha(0.4f);
                            break;
                        case 1: //index
                            rightIndex.setX(point[0] - rightIndex.getWidth()/2);
                            rightIndex.setY(point[1] - rightIndex.getHeight()/2);
                            rightIndex.setAlpha(0.4f);
                            break;
                        case 2: //middle
                            rightMiddle.setX(point[0] - rightMiddle.getWidth()/2);
                            rightMiddle.setY(point[1] - rightMiddle.getHeight()/2);
                            rightMiddle.setAlpha(0.4f);
                            break;
                        case 3: //ring
                            rightRing.setX(point[0] - rightRing.getWidth()/2);
                            rightRing.setY(point[1] - rightRing.getHeight()/2);
                            rightRing.setAlpha(0.4f);
                            break;
                        case 4: //pinky
                            rightPinky.setX(point[0] - rightPinky.getWidth()/2);
                            rightPinky.setY(point[1] - rightPinky.getHeight()/2);
                            rightPinky.setAlpha(0.4f);
                            break;
                        case 5: //thumb
                            rightThumb.setX(point[0] - rightThumb.getWidth()/2);
                            rightThumb.setY(point[1] - rightThumb.getHeight()/2);
                            rightThumb.setAlpha(0.4f);
                            break;
                        default:
                            break;
                    }
                    counter++;
                }
            }
        });

    }

    private float[] calculatePoint(float x, float y){
        float[] positionPixel = new float[]{x,y};
        float[] positionPoint = new float[2];
        Matrix streamMatrix = imageStream.getImageMatrix();
        streamMatrix.mapPoints(positionPoint, positionPixel);
        return positionPoint;
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        nodeMain=(NodeMainExecutorService)nodeMainExecutor;
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeMainExecutor.execute(imageStream, nodeConfiguration.setNodeName(STREAMING+"sub"));

        nodeMainExecutor.execute(positionNode, nodeConfiguration.setNodeName(TARGET_POINT));
        nodeMainExecutor.execute(graspNode, nodeConfiguration.setNodeName(GRASP));
        nodeMainExecutor.execute(rotationNode, nodeConfiguration.setNodeName(ROTATION));
        nodeMainExecutor.execute(emergencyNode, nodeConfiguration.setNodeName(EMERGENCY_STOP));
        nodeMainExecutor.execute(vsNode, nodeConfiguration.setNodeName(ENABLE_VS));
        nodeMainExecutor.execute(stringNode, nodeConfiguration.setNodeName(STRING_LOG));
        nodeMainExecutor.execute(interfaceNumberNode, nodeConfiguration.setNodeName(INTERFACE_NUMBER));
    }
}