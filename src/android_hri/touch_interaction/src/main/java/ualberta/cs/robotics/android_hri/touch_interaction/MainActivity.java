package ualberta.cs.robotics.android_hri.touch_interaction;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.ActionBarActivity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.Toast;

import java.io.IOException;

public class MainActivity extends ActionBarActivity {

    public static String ROS_MASTER = "";

    public static final float WORKSPACE_X_OFFSET = 0.2306f;
    public static final float WORKSPACE_WIDTH = 0.4889f;
    public static final float WORKSPACE_Y_OFFSET = 0.9100f;
    public static final float WORKSPACE_HEIGHT = 0.3546f;

    private EditText rosIP;
    private EditText rosPort;
    private ImageView interface_1;
    private ImageView interface_2;
    private ImageView interface_3;
    private ImageView interface_4;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        rosIP = (EditText) findViewById(R.id.editIP);
        rosPort = (EditText) findViewById(R.id.editPort);
        interface_1 = (ImageView) findViewById(R.id.imageViewPreviewController);
        interface_2 = (ImageView) findViewById(R.id.imageViewPreviewDragging);
        interface_3 = (ImageView) findViewById(R.id.imageViewPreviewGamepad);
        interface_4 = (ImageView) findViewById(R.id.imageViewPreviewLeapMotion);

        Button calibrationButton = (Button) findViewById(R.id.calibrationButton);
        calibrationButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startCalibrationActivity();
            }
        });

        Button controllerButton = (Button) findViewById(R.id.controllerButton);
        controllerButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startControllerActivity();
            }
        });
        interface_1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startControllerActivity();
            }
        });

        Button draggingButton = (Button) findViewById(R.id.draggingButton);
        draggingButton.setOnClickListener( new View.OnClickListener() {
            @Override
            public void onClick( View v ) {
                startDraggingActivity();
            }
        });
        interface_2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startDraggingActivity();
            }
        });

        Button gamepadButton = (Button) findViewById(R.id.gamepadButton);
        gamepadButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startGamepadActivity();
            }
        });
        interface_3.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startGamepadActivity();
            }
        });

        Button leapButton = (Button) findViewById(R.id.leapMotionButton);
        leapButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startLeapMotionActivity();
            }
        });
        interface_4.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startLeapMotionActivity();
            }
        });

    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    private void startCalibrationActivity(){
        if (isMasterValid()){
            Intent myIntent = new Intent(MainActivity.this, CalibrationActivity.class);
            MainActivity.this.startActivity(myIntent);
        }
    }

    private void startControllerActivity(){
        if (isMasterValid()){
            Intent myIntent = new Intent(MainActivity.this, ControllerActivity.class);
            MainActivity.this.startActivity(myIntent);
        }
    }

    private void startDraggingActivity(){
        if (isMasterValid()){
            Intent myIntent = new Intent(MainActivity.this, DraggingActivity.class);
            MainActivity.this.startActivity(myIntent);
        }
    }

    private void startGamepadActivity(){
        if (isMasterValid()){
            Intent myIntent = new Intent(MainActivity.this, GamepadActivity.class);
            MainActivity.this.startActivity(myIntent);
        }
    }

    private void startLeapMotionActivity(){
        if (isMasterValid()){
            Intent myIntent = new Intent(MainActivity.this, LeapMotionActivity.class);
            MainActivity.this.startActivity(myIntent);
        }
    }

    private boolean isMasterValid(){
        ROS_MASTER = "http://" + rosIP.getText().toString() + ":" + rosPort.getText().toString();
        int exit = pingHost(rosIP.getText().toString());
        if (exit!=0){
            Toast.makeText(getApplicationContext(), rosIP.getText().toString()+" is not reachable!!!",Toast.LENGTH_LONG).show();
            return false;
        }
        return true;
    }

    private static int pingHost(String host){
        int exit = -1;
        try {
            Runtime runtime = Runtime.getRuntime();
            Process proc = runtime.exec("ping -c 1 " + host);
            proc.waitFor();
            exit = proc.exitValue();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return exit;
    }

}
