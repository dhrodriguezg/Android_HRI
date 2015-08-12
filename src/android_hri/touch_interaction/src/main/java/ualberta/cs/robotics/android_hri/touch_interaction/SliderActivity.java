package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.WindowManager;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.ToggleButton;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.RosImageView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

import sensor_msgs.CompressedImage;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.MultiTouchArea;
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.TouchArea;
import ualberta.cs.robotics.android_hri.touch_interaction.node.ConfirmNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.SliderNode;
import ualberta.cs.robotics.android_hri.touch_interaction.node.TargetNode;


public class SliderActivity extends RosActivity {

	private static final String TAG = "SliderActivity";
    private MultiTouchArea targetHandler = null;
    private TouchArea sliderHandler = null;
    public final static int MAX_POWER = 100;

    private RosImageView<CompressedImage> imageStream;
    private ToggleButton toggleButton;
    private ImageView selectedArea;
    private ImageView sliderTouch;
    private ImageView sliderImage;
    private SliderNode sliderNode;
    private TargetNode targetNode;
    private ConfirmNode confirmNode;
    private boolean running = true;

    private int operator = 1;
    private boolean firstUpdate = true;

    public SliderActivity() {
        super("SliderActivity", "SliderActivity", URI.create(MainActivity.ROS_MASTER));
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {

    	Intent intent = getIntent();
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_slider);

        imageStream = (RosImageView<CompressedImage>) findViewById(R.id.imageViewCenter);
        imageStream.setTopicName("/camera/rgb/image_raw/compressed");
        imageStream.setMessageType("sensor_msgs/CompressedImage"); //% rostopic type /camera/rgb/image_raw
        imageStream.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        imageStream.setScaleType(ImageView.ScaleType.FIT_XY);

        selectedArea = (ImageView) findViewById(R.id.selectedArea);
        sliderTouch =  (ImageView) findViewById(R.id.sliderControl);
        sliderImage =  (ImageView) findViewById(R.id.imageSlider_p);

        toggleButton = (ToggleButton)findViewById(R.id.toggleButton) ;
        toggleButton.setOnCheckedChangeListener( new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton toggleButton, boolean isChecked) {
                confirmNode.setConfirm(isChecked);
            }
        });


        targetHandler = new MultiTouchArea(this, imageStream);
        sliderHandler = new TouchArea(this, sliderTouch);
        sliderNode = new SliderNode();
        targetNode = new TargetNode();
        confirmNode = new ConfirmNode();

        sliderImage.setTranslationY(sliderHandler.getHeight() / 2 - sliderImage.getHeight() / 2);
        sliderNode.setSliderValue(0.01f);

        //targetHandler.draw(BitmapFactory.decodeResource(getApplicationContext().getResources(), R.drawable.sample));



        Thread threadTarget = new Thread(){
            public void run(){
                while(running){
                    try {
                        Thread.sleep(100);
                        if(targetHandler.getLongClickX()>0){
                            updateTarget();
                        }
                    } catch (InterruptedException e) {
                        e.getStackTrace();
                    }
                }
            }
        };
        threadTarget.start();

        Thread threadSlider = new Thread(){
            public void run(){
                while(running){
                    try {
                        Thread.sleep(10);
                        if(sliderHandler.getSingleDragY() > 0  || sliderNode.getSliderValue() != 0){
                            updateSlider();
                        }
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

    public void updateTarget(){

        targetNode.setX(640*targetHandler.getLongClickX()/targetHandler.getWidth());
        targetNode.setY(480*targetHandler.getLongClickY()/targetHandler.getHeight());
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                selectedArea.setAlpha(0.5f);
                selectedArea.setTranslationX(targetHandler.getLongClickX() - selectedArea.getWidth() / 2);
                selectedArea.setTranslationY(targetHandler.getLongClickY() - selectedArea.getHeight() / 2);
            }
        });
    }
    public void updateSlider(){
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if(sliderHandler.isDetectingOneFingerGesture() && sliderHandler.getSingleDragY() > 0) {
                    sliderImage.setTranslationY(sliderHandler.getSingleDragY() - sliderImage.getHeight() / 2);
                    sliderNode.setSliderValue(sliderHandler.getSingleDragNormalizedY());
                } else {
                    sliderImage.setTranslationY(sliderHandler.getHeight() / 2 - sliderImage.getHeight() / 2);
                    sliderNode.setSliderValue(0);
                }
            }
        });
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeMainExecutor.execute(imageStream, nodeConfiguration.setNodeName("android/streaming"));
        nodeMainExecutor.execute(sliderNode, nodeConfiguration.setNodeName("android/slider"));
        nodeMainExecutor.execute(targetNode, nodeConfiguration.setNodeName("android/target"));
        nodeMainExecutor.execute(confirmNode, nodeConfiguration.setNodeName("android/confirm"));
    }



}
