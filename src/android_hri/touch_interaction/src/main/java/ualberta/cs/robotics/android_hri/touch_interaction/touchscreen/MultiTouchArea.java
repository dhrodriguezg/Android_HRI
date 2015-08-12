package ualberta.cs.robotics.android_hri.touch_interaction.touchscreen;

import android.app.Activity;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;

import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.gesture_detector.TwoFingerGestureDetector;

/**
 * Created by DarkNeoBahamut on 23/07/2015.
 */
public class MultiTouchArea extends TouchArea implements TwoFingerGestureDetector.OnTwoFingerGestureListener {

    private TwoFingerGestureDetector mTwoFingerGestureDetector;
    protected float angle;
    protected float scale;
    protected float doubleDragX;
    protected float doubleDragY;
    protected float doubleNormalizedDragX;
    protected float doubleNormalizedDragY;
    protected boolean detectingTwoFingerGesture = false;
    protected boolean multiChanged = false;

    public MultiTouchArea(Activity activity, ImageView view) {
        super(activity, view);
    }

    @Override
    protected void setupListener() {
        mTwoFingerGestureDetector = new TwoFingerGestureDetector(activity, this);
        view.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent event) {
                addOneFingerListener(view, event);
                addTwoFingerListener(view, event);
                return true;
            }
        });
    }

    protected void addTwoFingerListener(View view, MotionEvent event) {
        mTwoFingerGestureDetector.onTouchEvent(view, event);
    }

    @Override
    public void OnRotation(float mAngle) {
        angle = mAngle;
    }

    @Override
    public void OnDoubleDrag(float mX, float mY, float normalizedX, float normalizedY) {
        doubleDragX = mX;
        doubleDragY = mY;
        doubleNormalizedDragX = normalizedX;
        doubleNormalizedDragY = normalizedY;
    }

    @Override
    public void OnScale(float mScale) {
        scale=mScale;
    }

    @Override
    public void onTwoFingerGestureState(boolean detectingGesture) {
        detectingTwoFingerGesture =detectingGesture;
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

    public boolean isDetectingTwoFingerGesture() {
        return detectingTwoFingerGesture;
    }

    public void setDetectingTwoFingerGesture(boolean detectingTwoFingerGesture) {
        this.detectingTwoFingerGesture = detectingTwoFingerGesture;
    }


}
