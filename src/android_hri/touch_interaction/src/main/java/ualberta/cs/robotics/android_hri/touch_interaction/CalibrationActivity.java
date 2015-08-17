package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
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

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

import sensor_msgs.CompressedImage;
import ualberta.cs.robotics.android_hri.touch_interaction.node.ConfirmNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.RotationNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.TargetNode;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.MultiTouchArea;


public class CalibrationActivity extends RosActivity {

	private static final String TAG = "CalibrationActivity";
    private static final boolean debug = true;
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

    private TargetNode targetNode;
    private ConfirmNode confirmNode;
    private RotationNode rotationNode;
    private boolean firstRun=true;

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
            imageStream.setMessageType("sensor_msgs/CompressedImage");
        }else{
            //imageStream.setTopicName("/image_converter/output_video/compressed");
            imageStream.setTopicName("/camera/rgb/image_raw/compressed");
            imageStream.setMessageType("sensor_msgs/CompressedImage");
        }
        imageStream.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        imageStream.setScaleType(ImageView.ScaleType.MATRIX);

        confirmButton = (Button) findViewById(R.id.confirmButton);
        confirmButton.setOnClickListener( new View.OnClickListener() {
            @Override
            public void onClick( View v ) {
                //TODO
            }
        } );

        startSwitch = (Switch) findViewById(R.id.startSwitch);
        startSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked)
                    ;//TODO
            }
        });

        dragHandler = new MultiTouchArea(this, imageStream);
        dragHandler.enableScaling();
        dragHandler.enableOneFingerGestures();
        dragHandler.enableScroll();

        targetNode = new TargetNode();
        rotationNode = new RotationNode();
        confirmNode = new ConfirmNode();


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

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeMainExecutor.execute(imageStream, nodeConfiguration.setNodeName("android/streaming"));
        nodeMainExecutor.execute(targetNode, nodeConfiguration.setNodeName("android/target"));
        nodeMainExecutor.execute(confirmNode, nodeConfiguration.setNodeName("android/confirm"));
        nodeMainExecutor.execute(rotationNode, nodeConfiguration.setNodeName("android/rotation"));

    }

    public void updateTarget(){

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

                //Matrix previowsTMatrix = new Matrix();
                //previowsTMatrix.setScale(scale, scale);
                //previowsTMatrix.postTranslate(finalScaledCenteredX, finalScaledCenteredY);
                //imageStream.setImageMatrix(previowsTMatrix);

                float[] targetPoint = new float[2];
                float[] targetPixel = new float[2];
                float[] errorPoint = new float[2];
                //focusX = dragHandler.getScaleFocusX();
                //focusY = dragHandler.getScaleFocusY();
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

}
