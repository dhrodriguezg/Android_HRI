package ualberta.cs.robotics.android_hri.touch_interaction;

import android.app.Activity;
import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.widget.TextView;

import com.leapmotion.leap.Controller;

import ualberta.cs.robotics.android_hri.touch_interaction.utils.SampleListener;

public class LeapActivity extends Activity {

    private static final String TAG = "LeapActivity";
    private Controller mController;
    private SampleListener mListener;
    private TextView mView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_leap);
        mView = (TextView)findViewById(R.id.text_view);
        mView.setMovementMethod(new ScrollingMovementMethod());
    }

    @Override
    protected void onStart() {
        super.onStart();

        mController = new Controller();
        mListener = new SampleListener(this);
        mController.addListener(mListener);
    }

    @Override
    protected void onStop() {
        super.onStop();
        mController.removeListener(mListener);
        mController.delete();
        mListener.delete();
    }

    public void printLog(final String str) {
        runOnUiThread(new Runnable() {
            public void run() {
                mView.setText(str + "\n");
            }
        });
    }

    public void addLog(final String str) {
        runOnUiThread(new Runnable() {
            public void run() {
                mView.append(str + "\n");
            }
        });
    }

}
