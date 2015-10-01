package ualberta.cs.robotics.android_hri.touch_interaction.interfaces;

import android.content.Intent;
import android.graphics.Color;
import android.graphics.Matrix;
import android.os.Bundle;
import android.util.TypedValue;
import android.view.MenuItem;
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
import org.ros.android.view.VirtualJoystickView;

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
import ualberta.cs.robotics.android_hri.touch_interaction.node.TwistNode;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.TouchArea;

public class ScreenJoystickInterface extends RosActivity {

    private static final String TAG = "ScreenJoystickInterface";
    private static final String STREAMING= "/camera/rgb/image_raw/compressed";
    private static final String STREAMING_MSG = "sensor_msgs/CompressedImage";
    private static final String EMERGENCY_STOP = "/android/emergency_stop";
    private static final String TARGET_POINT="/android/target_point";
    private static final String INTERFACE_NUMBER="/android/interface_number";
    private static final String ENABLE_VS = "/android/enable_vs";
    private static final String POSITION= "/android/joystick_position";
    private static final String ROTATION= "/android/joystick_rotation";
    private static final String POSITION_ABS= "/android/position_abs";
    private static final String ROTATION_REL= "/android/rotation_rel";
    private static final String GRASP="/android/grasping_rel";
    private static final String TARGET= "/android/joystick_target_tmp";

    private NodeMainExecutorService nodeMain;

    private VirtualJoystickView mVirtualJoystickViewPosition;
    private VirtualJoystickView mVirtualJoystickViewRotation;
    private RosImageView<CompressedImage> imageStream;

    private PointNode targetPointNode;
    private PointNode positionPointNode;
    private PointNode rotationPointNode;

    private Float32Node graspNode;
    private Int32Node interfaceNumberNode;
    private BooleanNode emergencyNode;
    private BooleanNode vsNode;

    private TwistNode targetListenerNode;
    private TwistNode positionListenerNode;
    private TwistNode rotationListenerNode;

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

        mVirtualJoystickViewPosition = (VirtualJoystickView) findViewById(R.id.virtual_joystick_pos);
        mVirtualJoystickViewRotation = (VirtualJoystickView) findViewById(R.id.virtual_joystick_rot);
        mVirtualJoystickViewPosition.setHolonomic(true);
        mVirtualJoystickViewRotation.setHolonomic(true);

        imageStream = (RosImageView<CompressedImage>) findViewById(R.id.visualization);

        positionListenerNode = new TwistNode();
        positionListenerNode.subscribeTo(POSITION + "/cmd_vel");
        rotationListenerNode = new TwistNode();
        rotationListenerNode.subscribeTo(ROTATION + "/cmd_vel");

        graspNode = new Float32Node();
        graspNode.setPublishFreq(500);
        graspNode.publishTo(GRASP, true, 0);

        targetPointNode = new PointNode();
        targetPointNode.publishTo(TARGET_POINT, false, 10);

        positionPointNode =  new PointNode();
        positionPointNode.publishTo(POSITION_ABS, false, 50);

        rotationPointNode =  new PointNode();
        rotationPointNode.publishTo(ROTATION_REL, false, 50);

        interfaceNumberNode = new Int32Node();
        interfaceNumberNode.publishTo(INTERFACE_NUMBER, true, 0);
        interfaceNumberNode.setPublishFreq(100);
        interfaceNumberNode.setPublish_int(2);

        emergencyNode = new BooleanNode();
        emergencyNode.publishTo(EMERGENCY_STOP, true, 0);
        emergencyNode.setPublishFreq(100);
        emergencyNode.setPublish_bool(true);

        vsNode = new BooleanNode();
        vsNode.publishTo(ENABLE_VS, true, 0);
        vsNode.setPublish_bool(false);

        targetListenerNode = new TwistNode();
        targetListenerNode.subscribeTo(TARGET + "/cmd_vel");

        sliderTouch = (ImageView) findViewById(R.id.sliderControl);
        sliderImage = (ImageView) findViewById(R.id.imageSlider_p);
        targetImage = (ImageView) findViewById(R.id.imageTarget);
        int px = (int) TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, 120, getResources().getDisplayMetrics()); //convert pid to pixel
        int py = (int) TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, 6, getResources().getDisplayMetrics()); //convert pid to pixel
        RelativeLayout.LayoutParams params = (RelativeLayout.LayoutParams)targetImage.getLayoutParams();
        params.addRule(RelativeLayout.ALIGN_PARENT_BOTTOM);
        params.addRule(RelativeLayout.ALIGN_PARENT_RIGHT);
        params.rightMargin=px;
        params.bottomMargin=py;

        sliderHandler = new TouchArea(this, sliderTouch);
        sliderHandler.enableScroll();

        sliderImage.setTranslationY(sliderHandler.getHeight() / 2 - sliderImage.getHeight() / 2);
        graspNode.setPublish_float(0.01f); //it's just to init the thread below

        if(debug)
            imageStream.setTopicName("/usb_cam/image_raw/compressed");
        else
            imageStream.setTopicName(STREAMING);
        imageStream.setMessageType(STREAMING_MSG);
        imageStream.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        imageStream.setScaleType(ImageView.ScaleType.FIT_CENTER);

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

        Thread threadSlider = new Thread(){
            public void run(){
                while(running){
                    try {
                        Thread.sleep(10);
                        if(sliderHandler.getSingleDragY() > 0  || graspNode.getPublish_float() != 0){
                            updateSlider();
                        }
                        updatePosition();
                        updateRotation();
                        /*
                        if(positionListenerNode.hasReceivedMsg() || rotationListenerNode.hasReceivedMsg()){
                            vsNode.setPublish_bool(false);
                            positionListenerNode.setHasReceivedMsg(false);
                            rotationListenerNode.setHasReceivedMsg(false);
                            Log.d(TAG, String.format("MANUAL OPT [ %b ]", true));
                        }*/
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
        emergencyNode.setPublish_bool(true);
        vsNode.setPublish_bool(false);
        sliderImage.setTranslationY(sliderHandler.getHeight() / 2 - sliderImage.getHeight() / 2);
        running=true;
    }

    @Override
    protected void onPause() {
        emergencyNode.setPublish_bool(false);
        vsNode.setPublish_bool(false);
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
        /*
        switch (item.getItemId()) {
            case R.id.virtual_joystick_snap:
                if (!item.isChecked()) {
                    item.setChecked(true);
                    mVirtualJoystickViewPosition.EnableSnapping();
                    mVirtualJoystickViewRotation.EnableSnapping();
                    mVirtualJoystickViewTarget.EnableSnapping();
                } else {
                    item.setChecked(false);
                    mVirtualJoystickViewPosition.DisableSnapping();
                    mVirtualJoystickViewRotation.EnableSnapping();
                    mVirtualJoystickViewTarget.EnableSnapping();
                }
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
        */
        return super.onOptionsItemSelected(item);
    }

    private void updateSlider() {
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (sliderHandler.isDetectingOneFingerGesture() && sliderHandler.getSingleDragY() > 0) {
                    sliderImage.setTranslationY(sliderHandler.getSingleDragY() - sliderImage.getHeight() / 2);
                    graspNode.setPublish_float(sliderHandler.getSingleDragNormalizedY());
                } else {
                    sliderHandler.resetValuesOnRelease();
                    sliderImage.setTranslationY(sliderHandler.getHeight() / 2 - sliderImage.getHeight() / 2);
                    graspNode.setPublish_float(sliderHandler.getSingleDragNormalizedY());
                }
            }
        });
    }

    private void updatePosition() {

        float[] xy = positionListenerNode.getSubcribe_linear();
        final float x=targetImage.getX() - 2.f*xy[0];
        final float y=targetImage.getY() - 2.f*xy[1];

        float[] targetPoint = new float[]{x+targetImage.getWidth()/2 , y+targetImage.getHeight()/2};
        float[] targetPixel = new float[2];

        Matrix streamMatrix = new Matrix();
        imageStream.getImageMatrix().invert(streamMatrix);
        streamMatrix.mapPoints(targetPixel, targetPoint);
        if(!validTarget(targetPixel[0],targetPixel[1])){
            return;
        }

        positionPointNode.getPublish_point()[0] = MainActivity.WORKSPACE_Y_OFFSET - targetPixel[1]*MainActivity.WORKSPACE_HEIGHT/(float)imageStream.getDrawable().getIntrinsicHeight();
        positionPointNode.getPublish_point()[1] = MainActivity.WORKSPACE_X_OFFSET - targetPixel[0]*MainActivity.WORKSPACE_WIDTH/(float)imageStream.getDrawable().getIntrinsicWidth();
        positionPointNode.getPublish_point()[2] = 0;

        if(positionListenerNode.hasReceivedMsg())
            positionPointNode.publishNow();
        positionListenerNode.setHasReceivedMsg(false);

        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                targetImage.setX(x);
                targetImage.setY(y);
                targetImage.setAlpha(1.0f);
            }
        });
    }

    private void updateRotation() {

        float[] xy = rotationListenerNode.getSubcribe_linear();
        rotationPointNode.getPublish_point()[0] = xy[1];
        rotationPointNode.getPublish_point()[1] = xy[0];
        rotationPointNode.getPublish_point()[2] = 0;

        if(rotationListenerNode.hasReceivedMsg())
            rotationPointNode.publishNow();
        rotationListenerNode.setHasReceivedMsg(false);
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
        // Default ROS
        nodeMainExecutor.execute(mVirtualJoystickViewPosition, nodeConfiguration.setNodeName(POSITION));
        nodeMainExecutor.execute(mVirtualJoystickViewRotation, nodeConfiguration.setNodeName(ROTATION));
        nodeMainExecutor.execute(imageStream, nodeConfiguration.setNodeName(STREAMING+"sub"));

        //Custom
        nodeMainExecutor.execute(graspNode, nodeConfiguration.setNodeName(GRASP));
        nodeMainExecutor.execute(targetPointNode, nodeConfiguration.setNodeName(TARGET_POINT));
        nodeMainExecutor.execute(positionPointNode, nodeConfiguration.setNodeName(POSITION_ABS));
        nodeMainExecutor.execute(rotationPointNode, nodeConfiguration.setNodeName(ROTATION_REL));
        nodeMainExecutor.execute(emergencyNode, nodeConfiguration.setNodeName(EMERGENCY_STOP));
        nodeMainExecutor.execute(vsNode, nodeConfiguration.setNodeName(ENABLE_VS));
        nodeMainExecutor.execute(interfaceNumberNode, nodeConfiguration.setNodeName(INTERFACE_NUMBER));

        nodeMainExecutor.execute(targetListenerNode, nodeConfiguration.setNodeName(TARGET+"_sub"));
        nodeMainExecutor.execute(positionListenerNode, nodeConfiguration.setNodeName(POSITION+"_sub"));
        nodeMainExecutor.execute(rotationListenerNode, nodeConfiguration.setNodeName(ROTATION+"_sub"));
    }
}