package ualberta.cs.robotics.android_hri.touch_interaction.interfaces;

import android.content.Intent;
import android.graphics.Color;
import android.graphics.Matrix;
import android.os.Bundle;
import android.util.TypedValue;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.WindowManager;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.RelativeLayout;
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
import ualberta.cs.robotics.android_hri.touch_interaction.utils.Gamepad;
import ualberta.cs.robotics.android_hri.touch_interaction.utils.AndroidNode;
import ualberta.cs.robotics.android_hri.touch_interaction.topic.Int32Topic;
import ualberta.cs.robotics.android_hri.touch_interaction.topic.PointTopic;

public class GamepadInterface extends RosActivity {

    private static final String TAG = "GamepadInterface";
    private static final String NODE_NAME="/android/"+TAG.toLowerCase();

    private static final String STREAMING= "/camera/rgb/image_raw/compressed";
    private static final String STREAMING_MSG = "sensor_msgs/CompressedImage";
    private static final String EMERGENCY_STOP = "/android/emergency_stop";
    private static final String INTERFACE_NUMBER="/android/interface_number";
    private static final String ENABLE_VS = "/android/enable_vs";
    private static final String POSITION= "/android/position_abs";
    private static final String ROTATION= "/android/rotation_rel";
    private static final String GRASP="/android/grasping_rel";



    private NodeMainExecutorService nodeMain;

    private RosImageView<CompressedImage> imageStream;
    private ImageView targetImage;

    private AndroidNode androidNode;
    private BooleanTopic emergencyNode;
    private BooleanTopic vsNode;
    private PointTopic positionNode;
    private PointTopic rotationNode;
    private Float32Topic graspNode;
    private Int32Topic interfaceNumberNode;

    private Gamepad gamepad;

    private boolean running=true;
    private boolean debug=true;

    public GamepadInterface() {
        super(TAG, TAG, URI.create(MainActivity.ROS_MASTER));;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Intent intent = getIntent();
        setContentView(R.layout.interface_gamepad);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        imageStream = (RosImageView<CompressedImage>) findViewById(R.id.visualization);

        targetImage = (ImageView) findViewById(R.id.imageTarget);
        int px = (int) TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, 105, getResources().getDisplayMetrics()); //convert pid to pixel
        RelativeLayout.LayoutParams params = (RelativeLayout.LayoutParams)targetImage.getLayoutParams();
        params.addRule(RelativeLayout.ALIGN_PARENT_BOTTOM);
        params.addRule(RelativeLayout.ALIGN_PARENT_RIGHT);
        params.rightMargin=px;

        positionNode = new PointTopic();
        positionNode.publishTo(POSITION, false, 10);
        rotationNode = new PointTopic();
        rotationNode.publishTo(ROTATION, false, 10);

        graspNode = new Float32Topic();
        graspNode.setPublishingFreq(500);
        graspNode.publishTo(GRASP, true, 0);

        interfaceNumberNode = new Int32Topic();
        interfaceNumberNode.publishTo(INTERFACE_NUMBER, true, 0);
        interfaceNumberNode.setPublishingFreq(100);
        interfaceNumberNode.setPublisher_int(4);

        emergencyNode = new BooleanTopic();
        emergencyNode.publishTo(EMERGENCY_STOP, true, 0);
        emergencyNode.setPublishingFreq(100);
        emergencyNode.setPublisher_bool(true);

        vsNode = new BooleanTopic();
        vsNode.publishTo(ENABLE_VS, true, 0);
        vsNode.setPublisher_bool(false);

        androidNode = new AndroidNode(NODE_NAME);
        androidNode.addTopics(emergencyNode, vsNode, positionNode, rotationNode, graspNode, interfaceNumberNode);
        androidNode.addNodeMain(imageStream);

        gamepad = new Gamepad(this);
        if(gamepad.isAttached()){
            Toast.makeText(getApplicationContext(), "Gamepad found!", Toast.LENGTH_SHORT).show();
        }else{
            Toast.makeText(getApplicationContext(), "Gamepad not found! Leaving this activity now...", Toast.LENGTH_LONG).show();
            Thread exitActivity = new Thread(){
                public void run(){
                    try {
                        Thread.sleep(3000);
                        finish();
                    } catch (InterruptedException e) {
                        e.getStackTrace();
                    }
                }
            };
            exitActivity.start();
        }

        if(debug)
            imageStream.setTopicName("/usb_cam/image_raw/compressed");
        else
            imageStream.setTopicName(STREAMING);
        imageStream.setMessageType(STREAMING_MSG);
        imageStream.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        imageStream.setScaleType(ImageView.ScaleType.FIT_CENTER);

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

        Thread threadGamepad = new Thread(){
            public void run(){
                while(running){
                    try {
                        Thread.sleep(10);
                        sendGamepadValues();
                    } catch (InterruptedException e) {
                        e.getStackTrace();
                    }
                }
            }
        };
        threadGamepad.start();
    }

    @Override
    public void onResume() {
        super.onResume();
        emergencyNode.setPublisher_bool(true);
        vsNode.setPublisher_bool(false);
        running=true;
    }

    @Override
    protected void onPause() {
        emergencyNode.setPublisher_bool(false);
        vsNode.setPublisher_bool(false);
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
    public boolean dispatchGenericMotionEvent(MotionEvent motionEvent){
        super.dispatchGenericMotionEvent(motionEvent);
        return gamepad.dispatchGenericMotionEvent(motionEvent);
    }

    @Override
    public boolean dispatchKeyEvent(KeyEvent keyEvent){
        super.dispatchKeyEvent(keyEvent);
        return gamepad.dispatchKeyEvent(keyEvent);
    }

    private void sendGamepadValues(){
        float posX=-gamepad.getAxisValue(MotionEvent.AXIS_X);
        float posY=-gamepad.getAxisValue(MotionEvent.AXIS_Y);
        if(Math.abs(posX) > 0.1 || Math.abs(posY) > 0.1){
            //send positions
            final float x=targetImage.getX() - 2.f*posX;
            final float y=targetImage.getY() - 2.f*posY;
            float[] targetPoint = new float[]{x+targetImage.getWidth()/2 , y+targetImage.getHeight()/2};
            float[] targetPixel = new float[2];

            Matrix streamMatrix = new Matrix();
            imageStream.getImageMatrix().invert(streamMatrix);
            streamMatrix.mapPoints(targetPixel, targetPoint);
            if(validTarget(targetPixel[0],targetPixel[1])){
                positionNode.getPublisher_point()[0] = MainActivity.WORKSPACE_Y_OFFSET - targetPixel[1]*MainActivity.WORKSPACE_HEIGHT/(float)imageStream.getDrawable().getIntrinsicHeight();
                positionNode.getPublisher_point()[1] = MainActivity.WORKSPACE_X_OFFSET - targetPixel[0]*MainActivity.WORKSPACE_WIDTH/(float)imageStream.getDrawable().getIntrinsicWidth();
                positionNode.getPublisher_point()[2] = 0;
                positionNode.publishNow();

                this.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        targetImage.setX(x);
                        targetImage.setY(y);
                        targetImage.setAlpha(1.0f);
                    }
                });
            }

        }

        float rotX=-gamepad.getAxisValue(MotionEvent.AXIS_Z);
        float rotY=-gamepad.getAxisValue(MotionEvent.AXIS_RZ);
        if(Math.abs(rotX) > 0.1 || Math.abs(rotY) > 0.1){
            //send rotations
            rotationNode.getPublisher_point()[0]=rotY;
            rotationNode.getPublisher_point()[1]=rotX;
            rotationNode.publishNow();
        }

        float graspP=gamepad.getAxisValue(MotionEvent.AXIS_RTRIGGER);
        float graspN=-gamepad.getAxisValue(MotionEvent.AXIS_LTRIGGER);
        if(gamepad.isAttached()){
            graspNode.setPublisher_float(graspP + graspN);
        }
    }

    private boolean validTarget(float x, float y) {
        if (x < 0 || y < 0)
            return false;
        if (x > imageStream.getDrawable().getIntrinsicWidth() ||  y > imageStream.getDrawable().getIntrinsicHeight())
            return false;
        return true;
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        nodeMain=(NodeMainExecutorService)nodeMainExecutor;
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeMainExecutor.execute(androidNode, nodeConfiguration.setNodeName(androidNode.getName()));
    }
}
