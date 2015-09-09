package ualberta.cs.robotics.android_hri.touch_interaction;

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
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

import sensor_msgs.CompressedImage;
import ualberta.cs.robotics.android_hri.touch_interaction.node.BooleanNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.Float32Node;
import ualberta.cs.robotics.android_hri.touch_interaction.node.Int32Node;
import ualberta.cs.robotics.android_hri.touch_interaction.node.PointNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.StringNode;
import ualberta.cs.robotics.android_hri.touch_interaction.utils.LeapMotionListener;

public class LeapMotionActivity extends RosActivity implements LeapMotionListener.LeapMotionFrameListener{

	private static final String TAG = "LeapMotionActivity";
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

    private static final float WORKSPACE_WIDTH = 0.4889f;
    private static final float WORKSPACE_HEIGHT = 0.3822f;
    private static final float WORKSPACE_X_OFFSET = 0.2366f;
    private static final float WORKSPACE_Y_OFFSET = 0.9476f;

    private final int DISABLED = Color.RED;
    private final int ENABLED = Color.GREEN;
    private final int TRANSITION = Color.rgb(255,195,77); //orange
    private final int MAX_TASK_COUNTER = 120;

    private int selectCounter;
    private int moveCounter;
    private int rotateCounter;
    private int graspCounter;
    private int allCounter;
    private int confirmCounter;
    private boolean isConfirm=false;
    private boolean isEnable=true;

    private int lastTask = -1;
    private int currentTask = -1;

    private NodeMainExecutor nodeMain;
    private Controller mController;
    private LeapMotionListener mLeapMotionListener;

    private ToggleButton emergencyStop;
    private TextView statusGrasp;
    private TextView statusRotate;
    private TextView statusMove;
    private TextView statusSelect;
    private TextView statusAll;

    private static final float MAX_GRASP = 2.0f;

    private RosImageView<CompressedImage> imageStream;
    private TextView msgText;
    private TextView statusText;

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

    public LeapMotionActivity() {
        super(TAG, TAG, URI.create(MainActivity.ROS_MASTER));
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {

    	Intent intent = getIntent();
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_leapmotion);

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

        statusGrasp = (TextView) findViewById(R.id.statusGrasp);
        statusRotate = (TextView) findViewById(R.id.statusRotate);
        statusMove = (TextView) findViewById(R.id.statusMove);
        statusSelect = (TextView) findViewById(R.id.statusSelect);
        statusAll = (TextView) findViewById(R.id.statusAll);
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
                    imageStream.setBackgroundColor(Color.GREEN);
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
        nodeMain.shutdown();
        running=false;
        super.onDestroy();
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        if (id == R.id.action_settings) {
            return true;
        }
        return super.onOptionsItemSelected(item);
    }

    @Override
    public void onHands(final boolean hands) {
        runOnUiThread(new Runnable() {
            public void run() {
                if (hands) {
                    statusText.setText("OK!");
                    statusText.setBackgroundColor(Color.GREEN);
                } else {
                    statusText.setText("No Hands!");
                    statusText.setBackgroundColor(Color.RED);
                }
            }
        });
    }

    @Override
    public void onSelect(float x, float y, float z) {
        if(!isConfirm)
            return;
        if(((ColorDrawable)statusSelect.getBackground()).getColor() != ENABLED)
            return;
        positionNode.getPublish_point()[0]=imageStream.getDrawable().getIntrinsicWidth() *x/2f;
        positionNode.getPublish_point()[1]=imageStream.getDrawable().getIntrinsicHeight()*z/2f;
        float xt=imageStream.getDrawable().getIntrinsicWidth() *x/2f;
        float yt=imageStream.getDrawable().getIntrinsicHeight()*z/2f;

        float[] point = calculatePoint(xt, yt);
        final float xm = point[0] - rightIndex.getWidth()/2;
        final float ym = point[1] - rightIndex.getHeight()/2;
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                rightIndex.setX(xm);
                rightIndex.setY(ym);
                rightIndex.setAlpha(1.0f);
                //positionImage.setAlpha(0.f);
            }
        });
    }

    @Override
    public void onMove(float x, float y, float z) {
        if(!isConfirm)
            return;
        if(((ColorDrawable)statusMove.getBackground()).getColor() != ENABLED && ((ColorDrawable)statusAll.getBackground()).getColor() != ENABLED)
            return;
        vsNode.setPublish_bool(true);

        positionNode.getPublish_point()[0] = WORKSPACE_Y_OFFSET - z*WORKSPACE_HEIGHT;
        positionNode.getPublish_point()[1] = WORKSPACE_X_OFFSET - x*WORKSPACE_WIDTH;
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
        if(((ColorDrawable)statusRotate.getBackground()).getColor() != ENABLED && ((ColorDrawable)statusAll.getBackground()).getColor() != ENABLED)
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
        if(((ColorDrawable)statusGrasp.getBackground()).getColor() != ENABLED && ((ColorDrawable)statusAll.getBackground()).getColor() != ENABLED)
            return;
        float grasp = (1f-g)*2f;
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
                    case -1:
                        // nonvalid gesture (reset)
                        resetTaskSelection();
                        break;
                    case 0:
                        // all fingers closed (reset)
                        resetTaskSelection();
                        break;
                    case 9: //select
                        if (selectCounter > MAX_TASK_COUNTER) {
                            imageStream.setBackgroundColor(Color.TRANSPARENT);
                            isConfirm=false;
                            lastTask = task;
                            enableSelect();
                            return;
                        }
                        if (((ColorDrawable) statusSelect.getBackground()).getColor() != ENABLED)
                            statusSelect.setBackgroundColor(TRANSITION);
                        if (((ColorDrawable) statusMove.getBackground()).getColor() != ENABLED)
                            statusMove.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusRotate.getBackground()).getColor() != ENABLED)
                            statusRotate.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusGrasp.getBackground()).getColor() != ENABLED)
                            statusGrasp.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusAll.getBackground()).getColor() != ENABLED)
                            statusAll.setBackgroundColor(DISABLED);
                        selectCounter++;
                        break;
                    case 1: //move
                        if (moveCounter > MAX_TASK_COUNTER) {
                            imageStream.setBackgroundColor(Color.TRANSPARENT);
                            isConfirm=false;
                            lastTask = task;
                            enableMove();
                            return;
                        }
                        if (((ColorDrawable) statusMove.getBackground()).getColor() != ENABLED)
                            statusMove.setBackgroundColor(TRANSITION);
                        if (((ColorDrawable) statusSelect.getBackground()).getColor() != ENABLED)
                            statusSelect.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusRotate.getBackground()).getColor() != ENABLED)
                            statusRotate.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusGrasp.getBackground()).getColor() != ENABLED)
                            statusGrasp.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusAll.getBackground()).getColor() != ENABLED)
                            statusAll.setBackgroundColor(DISABLED);
                        moveCounter++;
                        break;
                    case 2: //rotate
                        if (rotateCounter > MAX_TASK_COUNTER) {
                            imageStream.setBackgroundColor(Color.TRANSPARENT);
                            isConfirm=false;
                            lastTask = task;
                            enableRotate();
                            return;
                        }
                        if (((ColorDrawable) statusRotate.getBackground()).getColor() != ENABLED)
                            statusRotate.setBackgroundColor(TRANSITION);
                        if (((ColorDrawable) statusSelect.getBackground()).getColor() != ENABLED)
                            statusSelect.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusMove.getBackground()).getColor() != ENABLED)
                            statusMove.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusGrasp.getBackground()).getColor() != ENABLED)
                            statusGrasp.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusAll.getBackground()).getColor() != ENABLED)
                            statusAll.setBackgroundColor(DISABLED);
                        rotateCounter++;
                        break;
                    case 3: //grasp
                        if (graspCounter > MAX_TASK_COUNTER) {
                            imageStream.setBackgroundColor(Color.TRANSPARENT);
                            isConfirm=false;
                            lastTask = task;
                            enableGrasp();
                            return;
                        }
                        if (((ColorDrawable) statusGrasp.getBackground()).getColor() != ENABLED)
                            statusGrasp.setBackgroundColor(TRANSITION);
                        if (((ColorDrawable) statusSelect.getBackground()).getColor() != ENABLED)
                            statusSelect.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusMove.getBackground()).getColor() != ENABLED)
                            statusMove.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusRotate.getBackground()).getColor() != ENABLED)
                            statusRotate.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusAll.getBackground()).getColor() != ENABLED)
                            statusAll.setBackgroundColor(DISABLED);
                        graspCounter++;
                        break;
                    case 4: //AAAAAALLLLL
                        if (allCounter > MAX_TASK_COUNTER) {
                            imageStream.setBackgroundColor(Color.TRANSPARENT);
                            isConfirm=false;
                            lastTask = task;
                            enableAll();
                            return;
                        }
                        if (((ColorDrawable) statusAll.getBackground()).getColor() != ENABLED)
                            statusAll.setBackgroundColor(TRANSITION);
                        if (((ColorDrawable) statusSelect.getBackground()).getColor() != ENABLED)
                            statusSelect.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusMove.getBackground()).getColor() != ENABLED)
                            statusMove.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusRotate.getBackground()).getColor() != ENABLED)
                            statusRotate.setBackgroundColor(DISABLED);
                        if (((ColorDrawable) statusGrasp.getBackground()).getColor() != ENABLED)
                            statusGrasp.setBackgroundColor(DISABLED);
                        allCounter++;
                        break;
                    case 5:
                        // all fingers opened (no task)
                        resetTaskSelection();
                        break;
                    case 6: //confirm
                        if (confirmCounter > MAX_TASK_COUNTER) {
                            confirmCounter=0;
                            isConfirm=!isConfirm;
                            lastTask = task;
                            if(isConfirm){
                                imageStream.setBackgroundColor(Color.GREEN);
                                Toast.makeText(getApplicationContext(), "Movement activated", Toast.LENGTH_LONG).show();
                            } else {
                                imageStream.setBackgroundColor(Color.TRANSPARENT);
                                Toast.makeText(getApplicationContext(), "Movement deactivated", Toast.LENGTH_LONG).show();
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
        selectCounter=0;
        moveCounter=0;
        rotateCounter=0;
        graspCounter=0;
        allCounter=0;
        isConfirm=false;
        statusSelect.setBackgroundColor(DISABLED);
        statusMove.setBackgroundColor(DISABLED);
        statusRotate.setBackgroundColor(DISABLED);
        statusGrasp.setBackgroundColor(DISABLED);
        statusAll.setBackgroundColor(DISABLED);
        imageStream.setBackgroundColor(Color.TRANSPARENT);
    }

    private void resetTaskSelection(){
        selectCounter=0;
        moveCounter=0;
        rotateCounter=0;
        graspCounter=0;
        allCounter=0;
        if (((ColorDrawable) statusSelect.getBackground()).getColor() != ENABLED)
            statusSelect.setBackgroundColor(DISABLED);
        if (((ColorDrawable) statusMove.getBackground()).getColor() != ENABLED)
            statusMove.setBackgroundColor(DISABLED);
        if (((ColorDrawable) statusRotate.getBackground()).getColor() != ENABLED)
            statusRotate.setBackgroundColor(DISABLED);
        if (((ColorDrawable) statusGrasp.getBackground()).getColor() != ENABLED)
            statusGrasp.setBackgroundColor(DISABLED);
        if (((ColorDrawable) statusAll.getBackground()).getColor() != ENABLED)
            statusAll.setBackgroundColor(DISABLED);
    }

    private void enableSelect(){
        statusSelect.setBackgroundColor(ENABLED);
        statusMove.setBackgroundColor(DISABLED);
        statusRotate.setBackgroundColor(DISABLED);
        statusGrasp.setBackgroundColor(DISABLED);
        statusAll.setBackgroundColor(DISABLED);
        moveCounter=0;
        rotateCounter=0;
        graspCounter=0;
        allCounter=0;
    }

    private void enableMove(){
        statusSelect.setBackgroundColor(DISABLED);
        statusMove.setBackgroundColor(ENABLED);
        statusRotate.setBackgroundColor(DISABLED);
        statusGrasp.setBackgroundColor(DISABLED);
        statusAll.setBackgroundColor(DISABLED);
        selectCounter=0;
        rotateCounter=0;
        graspCounter=0;
        allCounter=0;
    }

    private void enableRotate(){
        statusSelect.setBackgroundColor(DISABLED);
        statusMove.setBackgroundColor(DISABLED);
        statusRotate.setBackgroundColor(ENABLED);
        statusGrasp.setBackgroundColor(DISABLED);
        statusAll.setBackgroundColor(DISABLED);
        selectCounter=0;
        moveCounter=0;
        graspCounter=0;
        allCounter=0;
    }

    private void enableGrasp(){
        statusSelect.setBackgroundColor(DISABLED);
        statusMove.setBackgroundColor(DISABLED);
        statusRotate.setBackgroundColor(DISABLED);
        statusGrasp.setBackgroundColor(ENABLED);
        statusAll.setBackgroundColor(DISABLED);
        selectCounter=0;
        moveCounter=0;
        rotateCounter=0;
        allCounter=0;
    }

    private void enableAll(){
        statusSelect.setBackgroundColor(DISABLED);
        statusMove.setBackgroundColor(DISABLED);
        statusRotate.setBackgroundColor(DISABLED);
        statusGrasp.setBackgroundColor(DISABLED);
        statusAll.setBackgroundColor(ENABLED);
        selectCounter=0;
        moveCounter=0;
        rotateCounter=0;
        graspCounter=0;
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
        nodeMain=nodeMainExecutor;
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