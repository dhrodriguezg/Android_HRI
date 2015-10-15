package ualberta.cs.robotics.android_hri.touch_interaction.interfaces;

import android.content.Intent;
import android.graphics.Color;
import android.graphics.Matrix;
import android.os.Bundle;
import android.util.TypedValue;
import android.view.MenuItem;
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
import ualberta.cs.robotics.android_hri.touch_interaction.topic.Int32Topic;
import ualberta.cs.robotics.android_hri.touch_interaction.topic.PointTopic;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.TouchArea;
import ualberta.cs.robotics.android_hri.touch_interaction.utils.AndroidNode;
import ualberta.cs.robotics.android_hri.touch_interaction.utils.CustomVirtualJoystickView;

public class ScreenJoystickInterface extends RosActivity {

    private static final String TAG = "ScreenJoystickInterface";
    private static final String NODE_NAME="/android_"+TAG.toLowerCase();

    /*
    private static final String STREAMING= "/camera/rgb/image_raw/compressed";
    private static final String STREAMING_MSG = "sensor_msgs/CompressedImage";
    private static final String EMERGENCY_STOP = "/android/emergency_stop";
    private static final String INTERFACE_NUMBER="/android/interface_number";
    private static final String POSITION_ABS= "/android/position_abs";
    private static final String ROTATION_REL= "/android/rotation_rel";
    private static final String GRASP="/android/grasping_rel";
    */
    private NodeMainExecutorService nodeMain;

    private CustomVirtualJoystickView joystickPositionNodeMain;
    private CustomVirtualJoystickView joystickRotationNodeMain;
    private RosImageView<CompressedImage> imageStreamNodeMain;

    private AndroidNode androidNode;
    private BooleanTopic emergencyTopic;
    private Int32Topic interfaceNumberTopic;
    private PointTopic positionTopic;
    private PointTopic rotationTopic;
    private Float32Topic graspTopic;

    private TouchArea sliderHandler = null;
    private ImageView sliderTouch;
    private ImageView sliderImage;

    private ImageView targetImage;
    private boolean running=true;
    private boolean debug=true;

    public ScreenJoystickInterface() {
        super(TAG, TAG, URI.create(MainActivity.ROS_MASTER));;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Intent intent = getIntent();
        setContentView(R.layout.interface_screenjoystick);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        joystickPositionNodeMain = (CustomVirtualJoystickView) findViewById(R.id.virtual_joystick_pos);
        joystickRotationNodeMain = (CustomVirtualJoystickView) findViewById(R.id.virtual_joystick_rot);
        joystickPositionNodeMain.setHolonomic(true);
        joystickRotationNodeMain.setHolonomic(true);

        imageStreamNodeMain = (RosImageView<CompressedImage>) findViewById(R.id.visualization);

        graspTopic = new Float32Topic();
        graspTopic.setPublishingFreq(500);
        graspTopic.publishTo(getString(R.string.topic_graspingrel), true, 0);

        positionTopic =  new PointTopic();
        positionTopic.publishTo(getString(R.string.topic_positionabs), false, 50);

        rotationTopic =  new PointTopic();
        rotationTopic.publishTo(getString(R.string.topic_rotationrel), false, 50);

        interfaceNumberTopic = new Int32Topic();
        interfaceNumberTopic.publishTo(getString(R.string.topic_interfacenumber), true, 0);
        interfaceNumberTopic.setPublishingFreq(100);
        interfaceNumberTopic.setPublisher_int(2);

        emergencyTopic = new BooleanTopic();
        emergencyTopic.publishTo(getString(R.string.topic_emergencystop), true, 0);
        emergencyTopic.setPublishingFreq(100);
        emergencyTopic.setPublisher_bool(true);

        androidNode = new AndroidNode(NODE_NAME);
        androidNode.addTopics(emergencyTopic, positionTopic, rotationTopic, graspTopic, interfaceNumberTopic);
        androidNode.addNodeMain(imageStreamNodeMain);
        //In case you still need to publish the virtualjoysticks values, set the topicname and then add the joysticks to the AndroidNode
        //joystickPositionNodeMain.setTopicName(JOYPOS_TOPIC);
        //joystickRotationNodeMain.setTopicName(JOYROT_TOPIC);
        //androidNode.addNodeMains(joystickPositionNodeMain,joystickRotationNodeMain);

        sliderTouch = (ImageView) findViewById(R.id.sliderControl);
        sliderImage = (ImageView) findViewById(R.id.imageSlider_p);
        targetImage = (ImageView) findViewById(R.id.imageTarget);

        sliderHandler = new TouchArea(this, sliderTouch);
        sliderHandler.enableScroll();

        sliderImage.setTranslationY(sliderHandler.getHeight() / 2 - sliderImage.getHeight() / 2);
        graspTopic.setPublisher_float(0.01f); //it's just to init the thread below

        imageStreamNodeMain.setTopicName(getString(R.string.topic_streaming));
        imageStreamNodeMain.setMessageType(getString(R.string.topic_streaming_msg));
        imageStreamNodeMain.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        imageStreamNodeMain.setScaleType(ImageView.ScaleType.FIT_CENTER);

        ToggleButton emergencyStop = (ToggleButton)findViewById(R.id.emergencyButton) ;
        emergencyStop.setOnCheckedChangeListener( new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                if(isChecked){
                    Toast.makeText(getApplicationContext(), getString(R.string.emergency_on_msg), Toast.LENGTH_LONG).show();
                    imageStreamNodeMain.setBackgroundColor(Color.RED);
                    emergencyTopic.setPublisher_bool(false);
                }else{
                    Toast.makeText(getApplicationContext(), getString(R.string.emergency_off_msg), Toast.LENGTH_LONG).show();
                    imageStreamNodeMain.setBackgroundColor(Color.TRANSPARENT);
                    emergencyTopic.setPublisher_bool(true);
                }
            }
        });

        Thread threadSlider = new Thread(){
            public void run(){
                while(running){
                    try {
                        Thread.sleep(10);
                        updateGrasp();
                        updatePosition();
                        updateRotation();
                    } catch (InterruptedException e) {
                        e.getStackTrace();
                    }
                }
            }
        };
        threadSlider.start();

    }

    @Override
    public void onResume() {
        super.onResume();
        emergencyTopic.setPublisher_bool(true);
        sliderImage.setTranslationY(sliderHandler.getHeight() / 2 - sliderImage.getHeight() / 2);
        /*** The following code was created because sometimes the target image is not well positioned when the app is launched ***/
        int px = (int) TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, 120, getResources().getDisplayMetrics()); //convert pid to pixel
        int py = (int) TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, 6, getResources().getDisplayMetrics()); //convert pid to pixel
        RelativeLayout.LayoutParams params = (RelativeLayout.LayoutParams)targetImage.getLayoutParams();
        params.addRule(RelativeLayout.ALIGN_PARENT_BOTTOM);
        params.addRule(RelativeLayout.ALIGN_PARENT_RIGHT);
        params.rightMargin=px;
        params.bottomMargin=py;
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

    private void updateGrasp() {
        if(sliderHandler.getSingleDragY() == 0  && graspTopic.getPublisher_float() == 0)
            return;

        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (sliderHandler.isDetectingOneFingerGesture() && sliderHandler.getSingleDragY() > 0) {
                    sliderImage.setTranslationY(sliderHandler.getSingleDragY() - sliderImage.getHeight() / 2);
                    graspTopic.setPublisher_float(sliderHandler.getSingleDragNormalizedY());
                } else {
                    sliderHandler.resetValuesOnRelease();
                    sliderImage.setTranslationY(sliderHandler.getHeight() / 2 - sliderImage.getHeight() / 2);
                    graspTopic.setPublisher_float(sliderHandler.getSingleDragNormalizedY());
                }
            }
        });
    }

    private void updatePosition() {
        float posX=joystickPositionNodeMain.getAxisY();
        float posY=joystickPositionNodeMain.getAxisX();

        if(Math.abs(posX) > 0.1 || Math.abs(posY) > 0.1){
            //send positions
            final float x=targetImage.getX() - 2.f*posX;
            final float y=targetImage.getY() - 2.f*posY;
            float[] targetPoint = new float[]{x+targetImage.getWidth()/2 , y+targetImage.getHeight()/2};
            float[] targetPixel = new float[2];

            Matrix streamMatrix = new Matrix();
            imageStreamNodeMain.getImageMatrix().invert(streamMatrix);
            streamMatrix.mapPoints(targetPixel, targetPoint);
            if(validTarget(targetPixel[0],targetPixel[1])){
                positionTopic.getPublisher_point()[0] = MainActivity.WORKSPACE_Y_OFFSET - targetPixel[1]*MainActivity.WORKSPACE_HEIGHT/(float) imageStreamNodeMain.getDrawable().getIntrinsicHeight();
                positionTopic.getPublisher_point()[1] = MainActivity.WORKSPACE_X_OFFSET - targetPixel[0]*MainActivity.WORKSPACE_WIDTH/(float) imageStreamNodeMain.getDrawable().getIntrinsicWidth();
                positionTopic.getPublisher_point()[2] = 0;
                positionTopic.publishNow();
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
    }

    private void updateRotation() {
        float rotX=joystickRotationNodeMain.getAxisY();
        float rotY=joystickRotationNodeMain.getAxisX();
        if(Math.abs(rotX) > 0.1 || Math.abs(rotY) > 0.1){
            //send rotations
            rotationTopic.getPublisher_point()[0]=rotY;
            rotationTopic.getPublisher_point()[1]=rotX;
            rotationTopic.publishNow();
        }
    }

    private boolean validTarget(float x, float y) {
        if (x < 0 || y < 0)
            return false;
        if (x > imageStreamNodeMain.getDrawable().getIntrinsicWidth() ||  y > imageStreamNodeMain.getDrawable().getIntrinsicHeight())
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
