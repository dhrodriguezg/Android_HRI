package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
import android.graphics.Matrix;
import android.os.Bundle;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.VirtualJoystickView;

import org.ros.android.view.RosImageView;

import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

import sensor_msgs.CompressedImage;

import ualberta.cs.robotics.android_hri.touch_interaction.node.BooleanNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.Float32Node;
import ualberta.cs.robotics.android_hri.touch_interaction.node.PointNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.TwistNode;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.TouchArea;


public class ControllerActivity extends RosActivity {
	
	private static final String TAG = "ControllerActivity";
    private static final String CONFIRM_TARGET="/android/confirmTarget";
    private static final String TARGET_POINT="/android/target_point";
    private static final String GRASP="/android/grasp";
    private static final String POSITION= "/android/joystickPosition";
    private static final String ROTATION= "/android/joystickRotation";
    private static final String TARGET= "/android/joystickTarget";
    private static final String STREAMING= "/camera/rgb/image_raw/compressed";
    private static final String STREAMING_MSG = "sensor_msgs/CompressedImage";

    private VirtualJoystickView mVirtualJoystickViewPosition;
    private VirtualJoystickView mVirtualJoystickViewRotation;
    private VirtualJoystickView mVirtualJoystickViewTarget;
    private RosImageView<CompressedImage> imageStream;

    private PointNode targetPointNode;
    private Float32Node sliderNode;
    private BooleanNode confirmTargetNode;
    private TwistNode targetControlNode;

    private TouchArea sliderHandler = null;
    private ImageView sliderTouch;
    private ImageView sliderImage;

    private ImageView targetImage;
    private boolean running=true;
    private boolean debug=true;

    public ControllerActivity() {
        super(TAG, TAG, URI.create(MainActivity.ROS_MASTER));;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Intent intent = getIntent();
        setContentView(R.layout.activity_controller);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        mVirtualJoystickViewPosition = (VirtualJoystickView) findViewById(R.id.virtual_joystick_pos);
        mVirtualJoystickViewRotation = (VirtualJoystickView) findViewById(R.id.virtual_joystick_rot);
        mVirtualJoystickViewTarget = (VirtualJoystickView) findViewById(R.id.virtual_joystick_target);
        mVirtualJoystickViewPosition.setHolonomic(true);
        mVirtualJoystickViewRotation.setHolonomic(true);
        mVirtualJoystickViewTarget.setHolonomic(true);

        imageStream = (RosImageView<CompressedImage>) findViewById(R.id.visualization);

        sliderNode = new Float32Node(); //TODO
        sliderNode.publishTo(GRASP,true,0);
        targetPointNode = new PointNode();
        targetPointNode.publishTo(TARGET_POINT, true, 0);
        confirmTargetNode = new BooleanNode();
        confirmTargetNode.publishTo(CONFIRM_TARGET, true, 100);
        targetControlNode = new TwistNode();
        targetControlNode.subscribeTo(TARGET+"/cmd_vel");

        sliderTouch = (ImageView) findViewById(R.id.sliderControl);
        sliderImage = (ImageView) findViewById(R.id.imageSlider_p);
        targetImage = (ImageView) findViewById(R.id.imageTarget);

        sliderHandler = new TouchArea(this, sliderTouch);
        sliderHandler.enableScroll();

        sliderImage.setTranslationY(sliderHandler.getHeight() / 2 - sliderImage.getHeight() / 2);
        sliderNode.setPublish_float(0.01f); //it's just to init the thread below

        if(debug)
            imageStream.setTopicName("/usb_cam/image_raw/compressed");
        imageStream.setMessageType(STREAMING_MSG);
        imageStream.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        imageStream.setScaleType(ImageView.ScaleType.FIT_CENTER);

        Button confirmTarget = (Button) findViewById(R.id.confirmTarget);
        confirmTarget.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                confirmTargetNode.setPublish_bool(true);
                confirmTargetNode.publishNow();
            }
        });

        Thread threadSlider = new Thread(){
            public void run(){
                while(running){
                    try {
                        Thread.sleep(16);
                        if(sliderHandler.getSingleDragY() > 0  || sliderNode.getPublish_float() != 0){
                            updateSlider();
                        }
                        updateTarget();
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
        sliderImage.setTranslationY(sliderHandler.getHeight() / 2 - sliderImage.getHeight() / 2);
        running=true;
    }

    @Override
    protected void onPause()
    {
        super.onPause();
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        running=false;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
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
            case R.id.action_settings:
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    private void updateSlider(){
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if (sliderHandler.isDetectingOneFingerGesture() && sliderHandler.getSingleDragY() > 0) {
                    sliderImage.setTranslationY(sliderHandler.getSingleDragY() - sliderImage.getHeight() / 2);
                    sliderNode.setPublish_float(sliderHandler.getSingleDragNormalizedY());
                } else {
                    sliderHandler.resetValuesOnRelease();
                    sliderImage.setTranslationY(sliderHandler.getHeight() / 2 - sliderImage.getHeight() / 2);
                    sliderNode.setPublish_float(sliderHandler.getSingleDragNormalizedY());
                }
            }
        });
    }

    private void updateTarget(){

        if(!targetControlNode.hasReceivedMsg()){
            targetImage.setAlpha(1.0f);
        }

        float[] xy = targetControlNode.getSubcribe_linear();
        final float x=targetImage.getX() - 10.f*xy[0];
        final float y=targetImage.getY() - 10.f*xy[1];

        float[] targetPoint = new float[]{x+targetImage.getWidth()/2 , y+targetImage.getHeight()/2};
        float[] targetPixel = new float[2];

        Matrix streamMatrix = new Matrix();
        imageStream.getImageMatrix().invert(streamMatrix);
        streamMatrix.mapPoints(targetPixel, targetPoint);

        targetPointNode.getPublish_point()[0]=targetPixel[0];
        targetPointNode.getPublish_point()[1]=targetPixel[1];

        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                targetImage.setX(x);
                targetImage.setY(y);
            }
        });
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        // Default ROS
        nodeMainExecutor.execute(mVirtualJoystickViewPosition, nodeConfiguration.setNodeName(POSITION));
        nodeMainExecutor.execute(mVirtualJoystickViewRotation, nodeConfiguration.setNodeName(ROTATION));
        nodeMainExecutor.execute(mVirtualJoystickViewTarget, nodeConfiguration.setNodeName(TARGET));
        nodeMainExecutor.execute(imageStream, nodeConfiguration.setNodeName(STREAMING+"sub"));

        //Custom
        nodeMainExecutor.execute(sliderNode, nodeConfiguration.setNodeName(GRASP));
        nodeMainExecutor.execute(targetPointNode, nodeConfiguration.setNodeName(TARGET_POINT));
        nodeMainExecutor.execute(confirmTargetNode, nodeConfiguration.setNodeName(CONFIRM_TARGET));
        nodeMainExecutor.execute(targetControlNode, nodeConfiguration.setNodeName(TARGET+"sub"));
    }
}
