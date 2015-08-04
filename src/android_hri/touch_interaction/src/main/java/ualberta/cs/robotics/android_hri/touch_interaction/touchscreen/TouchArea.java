package ualberta.cs.robotics.android_hri.touch_interaction.touchscreen;

import android.app.Activity;
import android.graphics.Bitmap;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;

import ualberta.cs.robotics.android_hri.touch_interaction.utils.DoubleTapGestureDetector;
import ualberta.cs.robotics.android_hri.touch_interaction.utils.LongClickGestureDetector;
import ualberta.cs.robotics.android_hri.touch_interaction.utils.SingleDragGestureDetector;

/**
 * Created by Diego Rodriguez on 22/07/2015.
 */
public class TouchArea implements DoubleTapGestureDetector.OnDoubleTapGestureListener, SingleDragGestureDetector.OnSingleDragGestureListener, LongClickGestureDetector.OnLongClickGestureListener{

    protected SingleDragGestureDetector mSingleDragGestureDetector;
    protected DoubleTapGestureDetector mDoubleTapDetector;
    protected LongClickGestureDetector mLongClickGestureDetector;
    protected Activity activity;
    protected ImageView view;
    protected float singleX;
    protected float singleY;
    protected float doubleX;
    protected float doubleY;
    protected float longX;
    protected float longY;

    public TouchArea(Activity activity, ImageView view){
        mSingleDragGestureDetector = new SingleDragGestureDetector(this);
        mDoubleTapDetector = new DoubleTapGestureDetector(activity,this);
        mLongClickGestureDetector = new LongClickGestureDetector(activity,this);
        this.activity = activity;
        this.view = view;
        singleX = 0.f; singleY = 0.f;
        doubleX = 0.f; doubleY = 0.f;
        view.setVisibility(View.VISIBLE);
        setupListener();
    }

    protected void setupListener() {
        view.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent event) {
                if (event.getPointerCount() < 2) {
                    mSingleDragGestureDetector.onTouchEvent(view, event);
                    mDoubleTapDetector.onTouchEvent(view, event);
                    mLongClickGestureDetector.onTouchEvent(view, event);
                }
                return true;
            }
        });
    }

    public void draw(final Bitmap bitmap){
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                view.setImageBitmap(bitmap);
            }
        });
    }

    @Override
    public void OnSingleDrag(SingleDragGestureDetector singleDragGestureDetector) {
        singleX=singleDragGestureDetector.getX();
        singleY=singleDragGestureDetector.getY();
        Log.d("Log: SingleDrag", "X:" + singleX + " Y:" + singleY);
    }

    @Override
    public void OnDoubleTap(DoubleTapGestureDetector doubleTapGestureDetector) {
        doubleX=doubleTapGestureDetector.getX();
        doubleY=doubleTapGestureDetector.getY();
        Log.d("Log: DoubleTap", "X:" + doubleX + " Y:" + doubleY);
    }

    @Override
    public void OnLongClick(LongClickGestureDetector longClickGestureDetector) {
        longX=longClickGestureDetector.getX();
        longY=longClickGestureDetector.getY();
        Log.d("Log: LongClick", "X:" + longX + " Y:" + longY);
    }

    /*public float getX() {
        return view.getX();
    }

    public float getY() {
        return view.getY();
    }*/

    public float getSingleX() {
        return singleX;
    }

    public void setSingleX(float singleX) {
        this.singleX = singleX;
    }

    public float getSingleY() {
        return singleY;
    }

    public void setSingleY(float singleY) {
        this.singleY = singleY;
    }

    public float getWidth() {
        return view.getWidth();
    }

    public float getHeight() {
        return view.getHeight();
    }

}
