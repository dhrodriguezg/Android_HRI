package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
import android.graphics.Color;
import android.graphics.Matrix;
import android.os.Bundle;
import android.view.MenuItem;
import android.view.WindowManager;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.leapmotion.leap.Controller;

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
import ualberta.cs.robotics.android_hri.touch_interaction.node.StringNode;
import ualberta.cs.robotics.android_hri.touch_interaction.utils.LeapMotionListener;

public class LeapMotionActivity extends RosActivity implements LeapMotionListener.LeapMotionFrameListener{

	private static final String TAG = "LeapMotionActivity";
    private static final String STREAMING= "/image_converter/output_video/compressed";
    private static final String STREAMING_MSG = "sensor_msgs/CompressedImage";
    private static final String EMERGENCY_STOP = "/android/emergency_stop";
    private static final String ENABLE_VS = "/android/enable_vs";
    private static final String STRING_LOG = "/android/log";
    private static final String TARGET_POINT="/android/target_point";
    private static final String ROTATION= "/android/rotation_abs";
    private static final String GRASP="/android/grasping_abs";


    private NodeMainExecutor nodeMain;
    private Controller mController;
    private LeapMotionListener mLeapMotionListener;

    private ToggleButton emergencyStop;
    private ToggleButton selectButton;
    private ToggleButton moveButton;
    private ToggleButton rotateButton;
    private ToggleButton graspButton;

    private static final float MAX_GRASP = 2.0f;

    private RosImageView<CompressedImage> imageStream;
    private TextView msgText;
    private TextView statusText;
    private ImageView targetImage;
    private ImageView positionImage;

    private PointNode targetPointNode;
    private Float32Node graspNode;
    private PointNode rotationNode;
    private StringNode stringNode;
    private BooleanNode emergencyNode;
    private BooleanNode vsNode;

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
        targetImage = (ImageView) findViewById(R.id.targetView);
        positionImage = (ImageView) findViewById(R.id.positionView);

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

        selectButton = (ToggleButton)findViewById(R.id.selectButton);
        selectButton.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                if (isChecked) {
                    moveButton.setChecked(false);
                    rotateButton.setChecked(false);
                    graspButton.setChecked(false);
                }
            }
        });
        moveButton = (ToggleButton)findViewById(R.id.moveButton);
        moveButton.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                if (isChecked) {
                    selectButton.setChecked(false);
                    rotateButton.setChecked(false);
                    graspButton.setChecked(false);
                }
            }
        });
        rotateButton = (ToggleButton)findViewById(R.id.rotateButton);
        rotateButton.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                if (isChecked) {
                    moveButton.setChecked(false);
                    selectButton.setChecked(false);
                    graspButton.setChecked(false);
                }
            }
        });
        graspButton = (ToggleButton)findViewById(R.id.graspButton);
        graspButton.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                if (isChecked) {
                    moveButton.setChecked(false);
                    rotateButton.setChecked(false);
                    selectButton.setChecked(false);
                }
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
    public void onHands(final boolean hands) {
        runOnUiThread(new Runnable() {
            public void run() {
                if(hands){
                    statusText.setText("OK!");
                    statusText.setBackgroundColor(Color.GREEN);
                }else{
                    statusText.setText("No Hands!");
                    statusText.setBackgroundColor(Color.RED);
                }
            }
        });
    }

    @Override
    public void onSelect(float x, float y, float z) {
        if(!selectButton.isChecked())
            return;
        // vsNode.setPublish_bool(true); TODO
        targetPointNode.getPublish_point()[0]=imageStream.getDrawable().getIntrinsicWidth() *x/2f;
        targetPointNode.getPublish_point()[1]=imageStream.getDrawable().getIntrinsicHeight()*z/2f;
        float xt=imageStream.getDrawable().getIntrinsicWidth() *x/2f;
        float yt=imageStream.getDrawable().getIntrinsicHeight()*z/2f;
        //targetPointNode.publishNow(); //TODO
        float[] point = calculatePoint(xt, yt);
        final float xm = point[0] - targetImage.getWidth()/2;
        final float ym = point[1] - targetImage.getHeight()/2;
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                targetImage.setX(xm);
                targetImage.setY(ym);
                targetImage.setAlpha(1.0f);
                //positionImage.setAlpha(0.f);
            }
        });
    }

    @Override
    public void onMove(float x, float y, float z) {
        if(!moveButton.isChecked())
            return;
        vsNode.setPublish_bool(true);
        targetPointNode.getPublish_point()[0]=imageStream.getDrawable().getIntrinsicWidth() *x/2f;
        targetPointNode.getPublish_point()[1]=imageStream.getDrawable().getIntrinsicHeight()*z/2f;
        targetPointNode.publishNow();
        float[] point = calculatePoint(targetPointNode.getPublish_point()[0],targetPointNode.getPublish_point()[1]);
        final float xm = point[0] - positionImage.getWidth()/2;
        final float ym = point[1] - positionImage.getHeight()/2;
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                positionImage.setX(xm);
                positionImage.setY(ym);
                positionImage.setAlpha(0.4f);
                //targetImage.setAlpha(0.f);
            }
        });
    }

    @Override
    public void onRotate(float x, float y, float z) {
        if(!rotateButton.isChecked())
            return;
        vsNode.setPublish_bool(false);
        rotationNode.getPublish_point()[0]=x;
        rotationNode.getPublish_point()[1]=y;
        rotationNode.publishNow();
    }

    @Override
    public void onGrasping(float g) {
        if(!graspButton.isChecked())
            return;
        float grasp = (1f-g)*2f;
        graspNode.setPublish_float(grasp);
        graspNode.publishNow();
    }

    @Override
    public void onUpdateMsg(final String msg) {
        stringNode.setPublish_string(msg);
        stringNode.publishNow();
        if(!debug)
            return;
        runOnUiThread(new Runnable() {
            public void run() {
                msgText.setText(msg + "\n");
                //msgText.setAlpha(0.5f);
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

        nodeMainExecutor.execute(targetPointNode, nodeConfiguration.setNodeName(TARGET_POINT));
        nodeMainExecutor.execute(graspNode, nodeConfiguration.setNodeName(GRASP));
        nodeMainExecutor.execute(rotationNode, nodeConfiguration.setNodeName(ROTATION));
        nodeMainExecutor.execute(emergencyNode, nodeConfiguration.setNodeName(EMERGENCY_STOP));
        nodeMainExecutor.execute(vsNode, nodeConfiguration.setNodeName(ENABLE_VS));
        nodeMainExecutor.execute(stringNode, nodeConfiguration.setNodeName(STRING_LOG));
    }
}