package ualberta.cs.robotics.android_hri.touch_interaction.touchscreen;

import android.app.Activity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;

import ualberta.cs.robotics.android_hri.touch_interaction.utils.DoubleDragGestureDetector;
import ualberta.cs.robotics.android_hri.touch_interaction.utils.RotationGestureDetector;

/**
 * Created by DarkNeoBahamut on 23/07/2015.
 */
public class MultiTouchArea extends TouchArea implements RotationGestureDetector.OnRotationGestureListener, DoubleDragGestureDetector.OnDragGestureListener {

    private RotationGestureDetector mRotationDetector;
    private DoubleDragGestureDetector mDragDetector;
    protected float angle;
    protected float doubleDragX;
    protected float doubleDragY;
    protected float doubleNormalizedDragX;
    protected float doubleNormalizedDragY;
    protected boolean doubleDragRelease = false;
    protected boolean multiChanged = false;

    public MultiTouchArea(Activity activity, ImageView view) {
        super(activity, view);
    }

    @Override
    protected void setupListener() {
        mRotationDetector = new RotationGestureDetector(this);
        mDragDetector = new DoubleDragGestureDetector(this);
        view.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent event) {
                mRotationDetector.onTouchEvent(event);
                mDragDetector.onTouchEvent(view, event);
                if (event.getPointerCount() < 2) {
                    mSingleDragGestureDetector.onTouchEvent(view, event);
                    mDoubleTapDetector.onTouchEvent(view, event);
                    mLongClickGestureDetector.onTouchEvent(view, event);
                }
                return true;
            }
        });
    }

    @Override
    public void OnRotation(RotationGestureDetector rotationDetector) {
        angle = rotationDetector.getAngle();
        Log.d("Log: Rotation", "Angle:" + Float.toString(angle));
    }

    @Override
    public void OnDoubleDrag(DoubleDragGestureDetector doubleDragDetector) {
        doubleDragX = doubleDragDetector.getX();
        doubleDragY = doubleDragDetector.getY();
        doubleNormalizedDragX = doubleDragDetector.getNormalizedX();
        doubleNormalizedDragY = doubleDragDetector.getNormalizedY();
        doubleDragRelease = doubleDragDetector.isRelease();
        Log.d("Log: DoubleDrag", "X:" + doubleDragX + " Y:" + doubleDragY + " Active:" + doubleDragRelease);
    }

    public float getAngle() {
        return angle;
    }

    public void setAngle(float angle) {
        this.angle = angle;
    }

    public float getDoubleDragX() {
        return doubleDragX;
    }

    public void setDoubleDragX(float doubleDragX) {
        this.doubleDragX = doubleDragX;
    }

    public float getDoubleDragY() {
        return doubleDragY;
    }

    public void setDoubleDragY(float doubleDragY) {
        this.doubleDragY = doubleDragY;
    }

    public float getDoubleNormalizedDragX() {
        return doubleNormalizedDragX;
    }

    public void setDoubleNormalizedDragX(float doubleNormalizedDragX) {
        this.doubleNormalizedDragX = doubleNormalizedDragX;
    }

    public float getDoubleNormalizedDragY() {
        return doubleNormalizedDragY;
    }

    public void setDoubleNormalizedDragY(float doubleNormalizedDragY) {
        this.doubleNormalizedDragY = doubleNormalizedDragY;
    }

    public boolean isDoubleDragRelease() {
        return doubleDragRelease;
    }

    public void setDoubleDragRelease(boolean doubleDragRelease) {
        this.doubleDragRelease = doubleDragRelease;
    }
}
