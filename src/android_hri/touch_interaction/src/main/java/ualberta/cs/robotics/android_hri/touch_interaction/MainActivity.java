package ualberta.cs.robotics.android_hri.touch_interaction;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.ActionBarActivity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import java.io.IOException;

public class MainActivity extends ActionBarActivity {

    public static String ROS_MASTER = "";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        final EditText editIP = (EditText) findViewById(R.id.editIP);
        final EditText editPort = (EditText) findViewById(R.id.editPort);

        Button controllerWideButton = (Button) findViewById(R.id.controllerWideButton);
        controllerWideButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                ROS_MASTER = "http://" + editIP.getText().toString() + ":" + editPort.getText().toString();
                int exit = pingHost(editIP.getText().toString());
                if (exit==0){
                    Intent myIntent = new Intent(MainActivity.this, ControllerActivity.class);
                    myIntent.putExtra("layout", "wide");
                    MainActivity.this.startActivity(myIntent);
                }else{
                    Toast.makeText(getApplicationContext(),editIP.getText().toString()+" is not reachable!!!",Toast.LENGTH_LONG).show();
                }
            }
        });
        Button controllerTightButton = (Button) findViewById(R.id.controllerTightButton);
        controllerTightButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                ROS_MASTER = "http://" + editIP.getText().toString() + ":" + editPort.getText().toString();
                int exit = pingHost(editIP.getText().toString());
                if (exit==0){
                    Intent myIntent = new Intent(MainActivity.this, ControllerActivity.class);
                    myIntent.putExtra("layout", "tight");
                    MainActivity.this.startActivity(myIntent);
                }else{
                    Toast.makeText(getApplicationContext(),editIP.getText().toString()+" is not reachable!!!",Toast.LENGTH_LONG).show();
                }
            }
        });

        Button draggingButton = (Button) findViewById(R.id.draggingButton);
        draggingButton.setOnClickListener( new View.OnClickListener() {
            @Override
            public void onClick( View v ) {
                ROS_MASTER = "http://" + editIP.getText().toString() + ":" + editPort.getText().toString();
                int exit = pingHost(editIP.getText().toString());
                if (exit==0){
                    Intent myIntent = new Intent(MainActivity.this, DraggingActivity.class);
                    myIntent.putExtra("layout", "dragging");
                    MainActivity.this.startActivity(myIntent);
                }else {
                    Toast.makeText(getApplicationContext(), editIP.getText().toString() + " is not reachable!!!", Toast.LENGTH_LONG).show();
                }
            }
        } );

        Button sliderButton = (Button) findViewById(R.id.sliderButton);
        sliderButton.setOnClickListener( new View.OnClickListener() {
            @Override
            public void onClick( View v ) {
                ROS_MASTER = "http://" + editIP.getText().toString() + ":" + editPort.getText().toString();
                int exit = pingHost(editIP.getText().toString());
                if (exit==0){
                    Intent myIntent = new Intent(MainActivity.this, DraggingActivity.class);
                    myIntent.putExtra("layout", "slider");
                    MainActivity.this.startActivity(myIntent);
                }else {
                    Toast.makeText(getApplicationContext(), editIP.getText().toString() + " is not reachable!!!", Toast.LENGTH_LONG).show();
                }
            }
        } );

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

    public static int pingHost(String host){
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
