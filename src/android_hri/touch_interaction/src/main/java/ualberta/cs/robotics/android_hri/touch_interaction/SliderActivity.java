package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
import android.graphics.Bitmap;
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
import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.TouchArea;


public class SliderActivity extends RosActivity {

	private static final String TAG = "SliderActivity";
    private MultiTouchArea targetHandler = null;
    private TouchArea sliderHandler = null;
    public final static int MAX_POWER = 100;

    private RosImageView<CompressedImage> image;
    private ImageView selectedArea;
    private ImageView sliderTouch;
    private ImageView sliderImage;
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

        image = (RosImageView<CompressedImage>) findViewById(R.id.imageViewCenter);
        image.setTopicName("/camera/rgb/image_raw/compressed");
        image.setMessageType("sensor_msgs/CompressedImage"); //% rostopic type /camera/rgb/image_raw
        image.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        image.setScaleType(ImageView.ScaleType.FIT_XY);
        selectedArea = (ImageView) findViewById(R.id.selectedArea);
        sliderTouch =  (ImageView) findViewById(R.id.sliderControl);
        sliderImage =  (ImageView) findViewById(R.id.imageSlider_p);


        targetHandler = new MultiTouchArea(this, image);
        sliderHandler = new TouchArea(this, sliderTouch);
        //targetHandler.draw(BitmapFactory.decodeResource(getApplicationContext().getResources(), R.drawable.sample));



        Thread threadTarget = new Thread(){
            public void run(){
                while(running){
                    try {
                        if(targetHandler.getLongClickX()>0){
                            updateTarget();
                        }
                        Thread.sleep(100);
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
                        if(sliderHandler.getSingleDragY()>0){
                            updateSlider();
                        }
                        Thread.sleep(10);
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
        nodeMainExecutor.execute(image, nodeConfiguration.setNodeName("android/streaming"));
    }

    public void updateTarget(){
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                selectedArea.setAlpha(0.3f);
                selectedArea.setTranslationX(targetHandler.getLongClickX() - selectedArea.getWidth() / 2);
                selectedArea.setTranslationY(targetHandler.getLongClickY() - selectedArea.getHeight() / 2);
            }
        });
    }
    public void updateSlider(){
        this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                if(sliderHandler.isSingleDragRelease())
                    sliderImage.setTranslationY(sliderHandler.getHeight()/2 - sliderImage.getHeight() / 2);
                else
                    sliderImage.setTranslationY(sliderHandler.getSingleDragY() - sliderImage.getHeight() / 2);
            }
        });
    }

}
