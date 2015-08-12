package ualberta.cs.robotics.android_hri.touch_interaction.touchscreen;

import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.os.Vibrator;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;

import ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.gesture_detector.OneFingerGestureDetector;

/**
 * Created by Diego Rodriguez on 22/07/2015.
 */
public class TouchArea implements OneFingerGestureDetector.OnOneFingerGestureListener{

    protected OneFingerGestureDetector mOneFingerGestureDetector;
    protected Activity activity;
    protected ImageView view;
    protected Vibrator vibrator;

    protected boolean detectingOneFingerGesture = false;
    protected float singleDragX = -1.f;
    protected float singleDragY = -1.f;
    protected float singleDragNormalizedX = -1.f;
    protected float singleDragNormalizedY = -1.f;
    protected float doubleTapX = -1.f;
    protected float doubleTapY = -1.f;
    protected float doubleTapNormalizedX = -1.f;
    protected float doubleTapNormalizedY = -1.f;
    protected float longClickX = -1.f;
    protected float longClickY = -1.f;
    protected float longClickNormalizedX = -1.f;
    protected float longClickNormalizedY = -1.f;

    public TouchArea(Activity activity, ImageView view){
        mOneFingerGestureDetector = new OneFingerGestureDetector(activity,this);
        vibrator = (Vibrator) activity.getApplicationContext().getSystemService(Context.VIBRATOR_SERVICE);
        this.activity = activity;
        this.view = view;
        view.setVisibility(View.VISIBLE);
        setupListener();
    }

    protected void setupListener() {
        view.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent event) {
                addOneFingerListener(view, event);
                return true;
            }
        });
    }

    protected void addOneFingerListener(View view, MotionEvent event) {
        mOneFingerGestureDetector.onTouchEvent(view, event);
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
    public void onScrollDrag(float sX, float sY, float sdX, float sdY, float dX, float dY, float normalizedDX, float normalizedDY) {
        singleDragNormalizedX=normalizedDX;
        singleDragNormalizedY=normalizedDY;
        singleDragX=dX;
        singleDragY=dY;
    }

    @Override
    public void onDoubleTap(float dtX, float dtY, float normalizedDTX, float normalizedDTY) {
        doubleTapNormalizedX=normalizedDTX;
        doubleTapNormalizedY=normalizedDTY;
        doubleTapX=dtX;
        doubleTapY=dtY;
    }

    @Override
    public void onLongPress(float lpX, float lpY, float normalizedLPX, float normalizedLPY) {
        longClickNormalizedX=normalizedLPX;
        longClickNormalizedY=normalizedLPY;
        longClickX=lpX;
        longClickY=lpY;
        vibrator.vibrate(100);
    }

    @Override
    public void onSingleTap(float stX, float stY, float normalizedSTX, float normalizedSTY) {
        //unused...for now
    }

    @Override
    public void onFling(float fX, float fY, float fvX, float fvY) {
        //unused...for now
    }

    @Override
    public void onOneFingerGestureState(boolean detectingGesture) {
        detectingOneFingerGesture=detectingGesture;
        resetValuesOnRelease();
    }

    private void resetValuesOnRelease(){
        if(!detectingOneFingerGesture){
            singleDragNormalizedX=0;
            singleDragNormalizedY=0;
            singleDragX=0;
            singleDragY=0;
        }
    }


    public boolean isDetectingOneFingerGesture(){
        return detectingOneFingerGesture;
    }

    public float getSingleDragX() {
        return singleDragX;
    }

    public void setSingleDragX(float singleDragX) {
        this.singleDragX = singleDragX;
    }

    public float getSingleDragY() {
        return singleDragY;
    }

    public void setSingleDragY(float singleDragY) {
        this.singleDragY = singleDragY;
    }

    public float getSingleDragNormalizedX() {
        return singleDragNormalizedX;
    }

    public void setSingleDragNormalizedX(float singleDragNormalizedX) {
        this.singleDragNormalizedX = singleDragNormalizedX;
    }

    public float getSingleDragNormalizedY() {
        return singleDragNormalizedY;
    }

    public void setSingleDragNormalizedY(float singleDragNormalizedY) {
        this.singleDragNormalizedY = singleDragNormalizedY;
    }

    public float getDoubleTapX() {
        return doubleTapX;
    }

    public void setDoubleTapX(float doubleTapX) {
        this.doubleTapX = doubleTapX;
    }

    public float getDoubleTapY() {
        return doubleTapY;
    }

    public void setDoubleTapY(float doubleTapY) {
        this.doubleTapY = doubleTapY;
    }

    public float getDoubleTapNormalizedX() {
        return doubleTapNormalizedX;
    }

    public void setDoubleTapNormalizedX(float doubleTapNormalizedX) {
        this.doubleTapNormalizedX = doubleTapNormalizedX;
    }

    public float getDoubleTapNormalizedY() {
        return doubleTapNormalizedY;
    }

    public void setDoubleTapNormalizedY(float doubleTapNormalizedY) {
        this.doubleTapNormalizedY = doubleTapNormalizedY;
    }

    public float getLongClickX() {
        return longClickX;
    }

    public void setLongClickX(float longClickX) {
        this.longClickX = longClickX;
    }

    public float getLongClickY() {
        return longClickY;
    }

    public void setLongClickY(float longClickY) {
        this.longClickY = longClickY;
    }

    public float getLongClickNormalizedX() {
        return longClickNormalizedX;
    }

    public void setLongClickNormalizedX(float longClickNormalizedX) {
        this.longClickNormalizedX = longClickNormalizedX;
    }

    public float getLongClickNormalizedY() {
        return longClickNormalizedY;
    }

    public void setLongClickNormalizedY(float longClickNormalizedY) {
        this.longClickNormalizedY = longClickNormalizedY;
    }

    public float getWidth() {
        return view.getWidth();
    }

    public float getHeight() {
        return view.getHeight();
    }

}
