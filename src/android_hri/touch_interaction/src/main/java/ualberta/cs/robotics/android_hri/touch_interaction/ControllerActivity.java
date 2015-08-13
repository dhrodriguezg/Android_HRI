package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.view.MenuItem;
import android.view.WindowManager;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.Switch;
import android.widget.ToggleButton;

import org.ros.address.InetAddressFactory;
import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.RosActivity;
import org.ros.android.view.VirtualJoystickView;

import org.ros.android.view.RosImageView;

import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

import sensor_msgs.CompressedImage;

import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.TouchArea;
import ualberta.cs.robotics.android_hri.touch_interaction.node.Twist2JoyNode;


public class ControllerActivity extends RosActivity {
	
	private static final String TAG = "ControllerActivity";

    private VirtualJoystickView mVirtualJoystickViewPos;
    private VirtualJoystickView mVirtualJoystickViewRot;
    private RosImageView<CompressedImage> image;
    private Twist2JoyNode nodeMain;

	private ImageView imageView;
    private Switch graspingSwitch;
    private TouchArea posHandler = null;
    private TouchArea rotHandler = null;

    public final static int MAX_POWER = 100;
    
    private int operator = 1;
    private ToggleButton toggleButton = null;
    private ToggleButton lightButton = null;
    private boolean firstUpdate = true;

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

        mVirtualJoystickViewPos = (VirtualJoystickView) findViewById(R.id.virtual_joystick_pos);
        mVirtualJoystickViewRot = (VirtualJoystickView) findViewById(R.id.virtual_joystick_rot);
        graspingSwitch =  (Switch) findViewById(R.id.graspingSwitch);
        graspingSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(isChecked)
                    nodeMain.setGraspValue(1);
                else
                    nodeMain.setGraspValue(0);
            }
        });

        mVirtualJoystickViewPos.setHolonomic(true);
        mVirtualJoystickViewRot.setHolonomic(true);

        image = (RosImageView<CompressedImage>) findViewById(R.id.visualization);
        image.setTopicName("/camera/rgb/image_raw/compressed");
        image.setMessageType("sensor_msgs/CompressedImage"); //% rostopic type /camera/rgb/image_raw
        image.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        image.setScaleType(ImageView.ScaleType.FIT_CENTER);
        nodeMain = new Twist2JoyNode();
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
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeMainExecutor.execute(mVirtualJoystickViewPos, nodeConfiguration.setNodeName("android/joystickPos")); //  geometry_msgs/Twist   -->   sensor_msgs/Joy
        nodeMainExecutor.execute(mVirtualJoystickViewRot, nodeConfiguration.setNodeName("android/joystickRot")); //
        nodeMainExecutor.execute(image, nodeConfiguration.setNodeName("android/streaming"));
        nodeMainExecutor.execute(nodeMain, nodeConfiguration.setNodeName("android/joynode"));
    }

}
