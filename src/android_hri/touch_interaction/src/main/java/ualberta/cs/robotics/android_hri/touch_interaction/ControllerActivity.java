package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.view.MenuItem;
import android.view.WindowManager;
import android.widget.ImageView;
import android.widget.ToggleButton;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.VirtualJoystickView;

import org.ros.android.view.RosImageView;

import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

import sensor_msgs.CompressedImage;

import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.TouchArea;


public class ControllerActivity extends RosActivity {
	
	private static final String TAG = "ControllerActivity";

    private VirtualJoystickView mVirtualJoystickViewPos;
    private VirtualJoystickView mVirtualJoystickViewRot;
    private RosImageView<CompressedImage> image;
    private NodeMain nodeMain;


	private ImageView imageView;
    private TouchArea posHandler = null;
    private TouchArea rotHandler = null;

    public final static int MAX_POWER = 100;
    
    private int operator = 1;
    private ToggleButton toggleButton = null;
    private ToggleButton lightButton = null;
    private boolean firstUpdate = true;

    public ControllerActivity() {
        super("ControllerActivity", "ControllerActivity", URI.create(MainActivity.ROS_MASTER));;
    }

    //  roslaunch openni2.launch
    //  rosrun image_view image_view image:=/camera/rgb/image_raw
    //  rostopic echo /android/joystickPos/cmd_vel
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Intent intent = getIntent();
        String layout = intent.getStringExtra("layout");
        if (layout.equals("tight"))
            setContentView(R.layout.activity_controller_wide);
        else
            setContentView(R.layout.activity_controller_tight);

        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        mVirtualJoystickViewPos = (VirtualJoystickView) findViewById(R.id.virtual_joystick_pos);
        mVirtualJoystickViewRot = (VirtualJoystickView) findViewById(R.id.virtual_joystick_rot);
        mVirtualJoystickViewPos.setHolonomic(true);
        mVirtualJoystickViewRot.setHolonomic(true);

        image = (RosImageView<CompressedImage>) findViewById(R.id.visualization);
        image.setTopicName("/camera/rgb/image_raw/compressed");
        image.setMessageType("sensor_msgs/CompressedImage"); //% rostopic type /camera/rgb/image_raw
        image.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        nodeMain = new CustomNode();

        if (layout.equals("tight"))
            image.setScaleType(ImageView.ScaleType.FIT_CENTER);
        else
            image.setScaleType(ImageView.ScaleType.FIT_XY);

    }

    private void refreshView(final Bitmap bitmap){
        runOnUiThread(new Runnable() {
			@Override
			public void run() {
				imageView.setImageBitmap(bitmap);
			}
		});
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.virtual_joystick_snap:
                if (!item.isChecked()) {
                    item.setChecked(true);
                    mVirtualJoystickViewPos.EnableSnapping();
                    mVirtualJoystickViewRot.EnableSnapping();
                } else {
                    item.setChecked(false);
                    mVirtualJoystickViewPos.DisableSnapping();
                    mVirtualJoystickViewRot.EnableSnapping();
                }
                return true;
            case R.id.action_settings:
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        //image.init(nodeMainExecutor);
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeMainExecutor.execute(mVirtualJoystickViewPos, nodeConfiguration.setNodeName("android/joystickPos")); //  geometry_msgs/Twist   -->   sensor_msgs/Joy
        nodeMainExecutor.execute(mVirtualJoystickViewRot, nodeConfiguration.setNodeName("android/joystickRot")); //
        nodeMainExecutor.execute(image, nodeConfiguration.setNodeName("android/streaming"));
        nodeMainExecutor.execute(nodeMain, nodeConfiguration.setNodeName("android/joynode"));
    }

}
