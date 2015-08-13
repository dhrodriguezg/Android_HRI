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
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

import sensor_msgs.CompressedImage;
import ualberta.cs.robotics.android_hri.touch_interaction.MainActivity;
import ualberta.cs.robotics.android_hri.touch_interaction.R;
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

    private float newCorrectionX=0.f;
    private float newCorrectionY=0.f;

    private TargetNode targetNode;
    private ConfirmNode confirmNode;
    private RotationNode rotationNode;

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
             rostopic echo /android/joystickPos/cmd_vel
            */
            imageStream.setTopicName("/camera/rgb/image_raw/compressed");
            imageStream.setMessageType("sensor_msgs/CompressedImage");
        }else{
            imageStream.setTopicName("/image_converter/output_video/compressed");
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
        targetNode = new TargetNode();
        rotationNode = new RotationNode();
        confirmNode = new ConfirmNode();


        Thread threadTarget = new Thread(){
            public void run(){
                while(running){
                    try {
                        updateTarget();
                        Thread.sleep(34);
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


                float scale = dragHandler.getScale();
                float imageScaledWidth = imageStream.getDrawable().getIntrinsicWidth()*scale;
                float imageScaledWHeight = imageStream.getDrawable().getIntrinsicHeight()*scale;

                float imageScaledCenteredX = (imageStream.getWidth() - imageScaledWidth)/ 2;
                float imageScaledCenteredY = (imageStream.getHeight() - imageScaledWHeight)/ 2;

                float imageTraslationX = dragHandler.getSingleDragX()-dragHandler.getSingleIDragX();
                float imageTraslationY = dragHandler.getSingleDragY()-dragHandler.getSingleIDragY();

                float finalScaledCenteredX=imageScaledCenteredX + imageTraslationX + newCorrectionX;
                float finalScaledCenteredY=imageScaledCenteredY + imageTraslationY + newCorrectionY;

                Matrix m = new Matrix();
                m.setScale(scale, scale);
                m.postTranslate(finalScaledCenteredX, finalScaledCenteredY);
                imageStream.setImageMatrix(m);



                if(dragHandler.isDetectingOneFingerGesture()){
                    updateCenter=false;
                }else{
                    if(!updateCenter){
                        newCorrectionX += imageTraslationX;
                        newCorrectionY += imageTraslationY;
                        dragHandler.resetValuesOnRelease();
                    }
                    updateCenter=true;
                }

                float[] selectedPoint = new float[2];
                selectedPoint[0] = imageStream.getWidth()/2;
                selectedPoint[1] = imageStream.getHeight()/2;
                float[] selectedPixel = new float[2];
                Matrix minv =new Matrix();
                m.invert(minv);
                minv.mapPoints(selectedPixel, selectedPoint);

                Log.d(TAG, String.format("Selection [ %.4f %.4f ]", selectedPixel[0],selectedPixel[1]));
/*
                Matrix m = new Matrix();

                //m.postScale(dragHandler.getScale(), dragHandler.getScale(), dragHandler.getSingleIDragX()-dragHandler.getSingleDragX(),dragHandler.getSingleIDragY()-dragHandler.getSingleDragY());
                m.preTranslate(-dragHandler.getScaleFocusX(),-dragHandler.getScaleFocusY());
                m.postScale(dragHandler.getScale(), dragHandler.getScale());
                imageStream.setImageMatrix(m);

*/

            }
        });
    }

}
