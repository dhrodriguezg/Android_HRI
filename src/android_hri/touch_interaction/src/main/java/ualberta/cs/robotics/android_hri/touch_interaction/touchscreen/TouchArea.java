package ualberta.cs.robotics.android_hri.touch_interaction.touchscreen;

import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.os.Vibrator;
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
    protected Vibrator vibrator;
    protected boolean singleDragRelease = true;
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
        mSingleDragGestureDetector = new SingleDragGestureDetector(this);
        mDoubleTapDetector = new DoubleTapGestureDetector(activity,this);
        mLongClickGestureDetector = new LongClickGestureDetector(activity,this);
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
        singleDragNormalizedX=singleDragGestureDetector.getNormalizedX();
        singleDragNormalizedY=singleDragGestureDetector.getNormalizedX();
        singleDragX=singleDragGestureDetector.getX();
        singleDragY=singleDragGestureDetector.getY();
        singleDragRelease=singleDragGestureDetector.isRelease();
        Log.d("Log: SingleDrag", "X:" + singleDragX + " Y:" + singleDragY);
    }

    @Override
    public void OnDoubleTap(DoubleTapGestureDetector doubleTapGestureDetector) {
        doubleTapX=doubleTapGestureDetector.getX();
        doubleTapY=doubleTapGestureDetector.getY();
        doubleTapNormalizedX=doubleTapGestureDetector.getNormalizedX();
        doubleTapNormalizedY=doubleTapGestureDetector.getNormalizedY();
        Log.d("Log: DoubleTap", "X:" + doubleTapNormalizedX + " Y:" + doubleTapNormalizedY);
    }

    @Override
    public void OnLongClick(LongClickGestureDetector longClickGestureDetector) {
        longClickX=longClickGestureDetector.getX();
        longClickY=longClickGestureDetector.getY();
        longClickNormalizedX=longClickGestureDetector.getNormalizedX();
        longClickNormalizedY=longClickGestureDetector.getNormalizedY();
        vibrator.vibrate(100);
        Log.d("Log: LongClick", "X:" + longClickNormalizedX + " Y:" + longClickNormalizedY);
    }

    public boolean isSingleDragRelease(){
        return singleDragRelease;
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
