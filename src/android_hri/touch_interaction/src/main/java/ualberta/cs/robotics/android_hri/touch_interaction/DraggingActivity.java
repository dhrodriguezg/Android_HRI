package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.WindowManager;
import android.widget.ImageView;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

import sensor_msgs.CompressedImage;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.MultiTouchArea;
import ualberta.cs.robotics.android_hri.touch_interaction.node.ConfirmNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.RotationNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.TargetNode;


public class DraggingActivity extends RosActivity {
	
	private static final String TAG = "DraggingActivity";
    private MultiTouchArea dragHandler = null;
    public final static int MAX_POWER = 100;

    private RosImageView<CompressedImage> imageStream;
    private int operator = 1;
    private ImageView selectedArea;
    private boolean running = true;
    private TargetNode targetNode;
    private ConfirmNode confirmNode;
    private RotationNode rotationNode;

    public DraggingActivity() {
        super(TAG, TAG, URI.create(MainActivity.ROS_MASTER));
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {

    	Intent intent = getIntent();
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_dragging);

        imageStream = (RosImageView<CompressedImage>) findViewById(R.id.imageViewCenter);
        imageStream.setTopicName("/image_converter/output_video/compressed"); //"/camera/rgb/image_raw/compressed"
        imageStream.setMessageType("sensor_msgs/CompressedImage"); //% rostopic type /camera/rgb/image_raw
        imageStream.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        imageStream.setScaleType(ImageView.ScaleType.FIT_XY);

        selectedArea = (ImageView) findViewById(R.id.selectedArea);

        dragHandler = new MultiTouchArea(this, imageStream);
        targetNode = new TargetNode();
        rotationNode = new RotationNode();
        confirmNode = new ConfirmNode();

        Thread threadTarget = new Thread(){
            public void run(){
                while(running){
                    try {
                        if(dragHandler.getDoubleDragX()>0){
                            updateTarget();
                        }
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.getStackTrace();
                    }
                }
            }
        };
        threadTarget.start();

        Thread threadRotation = new Thread(){
            public void run(){
                while(running){
                    try {
                        rotationNode.setRotationValue(dragHandler.getAngle());
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.getStackTrace();
                    }
                }
            }
        };
        threadRotation.start();

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

        targetNode.setX(imageStream.getDrawable().getIntrinsicWidth() * dragHandler.getDoubleDragX() / dragHandler.getWidth());
        targetNode.setY(imageStream.getDrawable().getIntrinsicHeight() * dragHandler.getDoubleDragY() / dragHandler.getHeight());
        confirmNode.setConfirm(dragHandler.isDetectingTwoFingerGesture());
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                selectedArea.setAlpha(0.5f);
                selectedArea.setTranslationX(dragHandler.getDoubleDragX() - selectedArea.getWidth() / 2);
                selectedArea.setTranslationY(dragHandler.getDoubleDragY() - selectedArea.getHeight() / 2);
            }
        });
    }

}
