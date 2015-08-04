package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
import android.graphics.BitmapFactory;
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


public class DraggingActivity extends RosActivity {
	
	private static final String TAG = "DraggingActivity";
    private MultiTouchArea touchHandler = null;
    public final static int MAX_POWER = 100;

    private RosImageView<CompressedImage> image;
    
    private int operator = 1;
    private boolean firstUpdate = true;

    public DraggingActivity() {
        super("DraggingActivity", "DraggingActivity", URI.create(MainActivity.ROS_MASTER));
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {

    	Intent intent = getIntent();
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_dragging);

        image = (RosImageView<CompressedImage>) findViewById(R.id.imageViewCenter);
        image.setTopicName("/camera/rgb/image_raw/compressed");
        image.setMessageType("sensor_msgs/CompressedImage"); //% rostopic type /camera/rgb/image_raw
        image.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        image.setScaleType(ImageView.ScaleType.FIT_XY);
        image.getDrawable().getBounds(); //TODO

        touchHandler = new MultiTouchArea(this, image);
        //touchHandler.draw(BitmapFactory.decodeResource(getApplicationContext().getResources(), R.drawable.sample));
		operator = 2;

    }

    @Override
    public void onResume() {
        super.onResume();
    }
    
    @Override
    protected void onPause()
    {
    	super.onPause();
    }
    
    @Override
    public void onDestroy() {
        super.onDestroy();
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

}
